/* SPDX-License-Identifier: MIT */
#include "riptide/gpu_stream.h"
#include <stdlib.h>
#include <string.h>
#include <limits.h>
static uint64_t next_serial;
static bool reserve(void **p, unsigned *capacity, unsigned needed, size_t stride) {
    if (needed <= *capacity)
        return true;
    unsigned n = *capacity ? *capacity : 1024;
    while (n < needed) {
        if (n > UINT_MAX / 2)
            return false;
        n *= 2;
    }
    if ((size_t)n > SIZE_MAX / stride)
        return false;
    void *v = realloc(*p, (size_t)n * stride);
    if (!v)
        return false;
    *p = v;
    *capacity = n;
    return true;
}
static bool append(RiptideGpuIndices *a, const uint32_t *v, unsigned count) {
    if (!count)
        return true;
    if (count > UINT_MAX - a->count)
        return false;
    if (!reserve((void **)&a->v, &a->capacity, a->count + count, sizeof *a->v))
        return false;
    memcpy(a->v + a->count, v, count * sizeof *v);
    a->count += count;
    return true;
}
RiptideGpuStream *riptide_gpu_stream_new(void) {
    RiptideGpuStream *s = calloc(1, sizeof *s);
    if (!s)
        return NULL;
    s->cache = calloc(GPU_CACHE_ENTRIES, sizeof *s->cache);
    if (!s->cache) {
        free(s);
        return NULL;
    }
    s->serial = ++next_serial;
    s->revision = 1;
    return s;
}
void riptide_gpu_stream_free(RiptideGpuStream *s) {
    if (!s)
        return;
    for (int p = 0; p < 4; p++) {
        free(s->pass[p].v);
        free(s->cached[p].v);
    }
    free(s->dynamic);
    free(s->fixed);
    for (unsigned i = 0; i < GPU_CACHE_ENTRIES; i++)
        free(s->cache[i].indices);
    free(s->free_spans);
    free(s->retired);
    free(s->dirty);
    free(s->cache);
    free(s);
}
static void invalidate(RiptideGpuStream *s) {
    for (unsigned i = 0; i < GPU_CACHE_ENTRIES; i++)
        free(s->cache[i].indices);
    memset(s->cache, 0, GPU_CACHE_ENTRIES * sizeof *s->cache);
    s->free_count = s->retired_count = s->dirty_count = s->sweep = 0;
    s->index_bytes = 0;
    s->fixed_count = s->stale = 0;
    s->invalidations++;
    for (int p = 0; p < 4; p++)
        s->cached[p].count = 0;
    s->revision++;
}
/* Stable primitive addresses: retire a block only after the current view is
   consumed, then reuse holes. Live geometry is never moved/re-uploaded wholesale. */
static bool span_add(RiptideGpuSpan **spans, unsigned *count, unsigned *capacity, unsigned first,
                     unsigned size) {
    if (!size)
        return true;
    unsigned end = first + size, at = 0;
    while (at < *count && (*spans)[at].first + (*spans)[at].size < first)
        at++;
    unsigned stop = at;
    while (stop < *count && (*spans)[stop].first <= end) {
        if ((*spans)[stop].first < first)
            first = (*spans)[stop].first;
        unsigned next = (*spans)[stop].first + (*spans)[stop].size;
        if (next > end)
            end = next;
        stop++;
    }
    if (!reserve((void **)spans, capacity, *count + 1, sizeof **spans))
        return false;
    memmove(*spans + at + 1, *spans + stop, (*count - stop) * sizeof **spans);
    (*spans)[at] = (RiptideGpuSpan){first, end - first};
    *count = *count - (stop - at) + 1;
    return true;
}
static void retire(RiptideGpuStream *s, RiptideGpuCache *c) {
    if (!c->valid)
        return;
    if (!span_add(&s->retired, &s->retired_count, &s->retired_capacity, c->first, c->size))
        s->failed = true;
    s->stale += c->size;
    c->valid = false;
}
static void reclaim(RiptideGpuStream *s) {
    /* Bounded age sweep; two camera variants remain cached during split-screen. */
    if (s->fixed_count > 327680)
        for (unsigned n = 0; n < 512; n++) {
            RiptideGpuCache *c = &s->cache[s->sweep++ % GPU_CACHE_ENTRIES];
            if (c->valid && s->epoch - c->used > 420)
                retire(s, c);
        }
    if (s->retired_count)
        s->compactions++; /* historical diagnostic: reclamation passes */
    for (unsigned i = 0; i < s->retired_count; i++) {
        RiptideGpuSpan b = s->retired[i];
        if (!span_add(&s->free_spans, &s->free_count, &s->free_capacity, b.first, b.size))
            s->failed = true;
    }
    s->retired_count = s->stale = 0;
    if (s->free_count) {
        RiptideGpuSpan *last = &s->free_spans[s->free_count - 1];
        if (last->first + last->size == s->fixed_count) {
            s->fixed_count = last->first;
            s->free_count--;
        }
    }
}
void riptide_gpu_stream_reset(RiptideGpuStream *s) {
    if (!s)
        return;
    if (s->recording) {
        s->failed = true;
        return;
    }
    s->epoch++;
    reclaim(s);
    s->dynamic_count = s->hits = s->misses = s->recording = s->dropped = 0;
    for (int p = 0; p < 4; p++)
        s->pass[p].count = 0;
}
size_t riptide_gpu_stream_bytes(const RiptideGpuStream *s) {
    if (!s)
        return 0;
    size_t bytes = sizeof *s + GPU_CACHE_ENTRIES * sizeof *s->cache;
    bytes += ((size_t)s->fixed_capacity + s->dynamic_capacity) * sizeof(RiptideGpuPrimitive);
    for (int p = 0; p < 4; p++)
        bytes += ((size_t)s->pass[p].capacity + s->cached[p].capacity) * sizeof(uint32_t);
    bytes += s->index_bytes; /* O(1): no 40,960-entry walk on every GPU draw. */
    bytes +=
        ((size_t)s->free_capacity + s->retired_capacity + s->dirty_capacity) * sizeof(RiptideGpuSpan);
    return bytes;
}
unsigned riptide_gpu_stream_capacity(unsigned current, unsigned needed) {
    if (needed <= current)
        return current;
    unsigned n = current ? current : 16384;
    while (n < needed) {
        unsigned add = n / 2;
        if (n > UINT_MAX - add - 16383)
            return 0;
        n = (n + add + 16383u) & ~16383u;
    }
    return n;
}
static bool reserve_fixed(RiptideGpuStream *s, unsigned needed) {
    if (needed <= s->fixed_capacity)
        return true;
    unsigned n = riptide_gpu_stream_capacity(s->fixed_capacity, needed);
    if (n < needed || n > UINT_MAX / sizeof *s->fixed)
        return false;
    void *v = realloc(s->fixed, (size_t)n * sizeof *s->fixed);
    if (!v)
        return false;
    s->fixed = v;
    s->fixed_capacity = n;
    return true;
}
void riptide_gpu_stream_scene(RiptideGpuStream *s, uint64_t key) {
    if (!s || s->scene_key == key)
        return;
    invalidate(s);
    /* New cities need not retain a previous city's high-water allocation. */
    free(s->fixed);
    s->fixed = NULL;
    s->fixed_capacity = 0;
    for (int p = 0; p < 4; p++) {
        free(s->cached[p].v);
        s->cached[p] = (RiptideGpuIndices){0};
    }
    s->scene_key = key;
    riptide_gpu_stream_reset(s);
}
void riptide_gpu_stream_emit(RiptideGpuStream *s, int pass, const RiptideGpuPrimitive *v) {
    if (!s)
        return;
    if (!v || pass < 0 || pass >= 4) {
        s->failed = true;
        s->dropped++;
        return;
    }
    if (s->failed) {
        s->dropped++;
        return;
    }
    bool fixed = s->recording != 0;
    RiptideGpuPrimitive **data = fixed ? &s->fixed : &s->dynamic;
    unsigned *count = fixed ? &s->fixed_count : &s->dynamic_count;
    unsigned *cap = fixed ? &s->fixed_capacity : &s->dynamic_capacity;
    if (!(fixed ? reserve_fixed(s, *count + 1)
                : reserve((void **)data, cap, *count + 1, sizeof **data)))
        goto error;
    uint32_t index = (*count) | (fixed ? 0 : GPU_DYNAMIC_BIT);
    (*data)[(*count)++] = *v;
    if (!append(&s->pass[pass], &index, 1))
        goto error;
    if (fixed && !append(&s->cached[pass], &index, 1))
        goto error;
    return;
error:
    s->failed = true;
    s->dropped++;
}
bool riptide_gpu_cache_begin(RiptideGpuStream *s, unsigned slot, uint64_t key) {
    if (!s || s->cache_disabled)
        return false;
    if (slot >= GPU_CACHE_SLOTS || s->recording || s->failed) {
        s->failed = true;
        return false;
    }
    unsigned base = slot * GPU_CACHE_WAYS, chosen = base;
    for (unsigned way = 0; way < GPU_CACHE_WAYS; way++) {
        unsigned at = base + way;
        RiptideGpuCache *c = &s->cache[at];
        if (c->valid && c->key == key) {
            for (int p = 0; p < 4; p++)
                if (c->count[p] && !append(&s->pass[p], c->indices + c->start[p], c->count[p]))
                    s->failed = true;
            c->used = s->epoch;
            s->hits++;
            return true;
        }
        if (!c->valid || (s->cache[chosen].valid && c->used < s->cache[chosen].used))
            chosen = at;
    }
    RiptideGpuCache *c = &s->cache[chosen];
    retire(s, c);
    c->first = s->fixed_count;
    c->size = 0;
    c->used = s->epoch;
    s->misses++;
    c->key = key;
    c->valid = false;
    for (int p = 0; p < 4; p++)
        s->cached[p].count = 0;
    s->recording = chosen + 1;
    return false;
}
void riptide_gpu_cache_end(RiptideGpuStream *s) {
    if (!s || !s->recording)
        return;
    RiptideGpuCache *c = &s->cache[s->recording - 1];
    c->size = s->fixed_count - c->first;
    unsigned dest = c->first, best = s->free_count;
    for (unsigned i = 0; i < s->free_count; i++)
        if (s->free_spans[i].size >= c->size &&
            (best == s->free_count || s->free_spans[i].size < s->free_spans[best].size))
            best = i;
    if (c->size && best < s->free_count) {
        RiptideGpuSpan *hole = &s->free_spans[best];
        dest = hole->first;
        memcpy(s->fixed + dest, s->fixed + c->first, c->size * sizeof *s->fixed);
        hole->first += c->size;
        hole->size -= c->size;
        if (!hole->size) {
            memmove(hole, hole + 1, (s->free_count - best - 1) * sizeof *hole);
            s->free_count--;
        }
        s->fixed_count = c->first;
    }
    unsigned old_capacity = c->capacity;
    if (!span_add(&s->dirty, &s->dirty_count, &s->dirty_capacity, dest, c->size) ||
        !reserve((void **)&c->indices, &c->capacity, c->size, sizeof *c->indices))
        s->failed = true;
    s->index_bytes += (size_t)(c->capacity - old_capacity) * sizeof(uint32_t);
    unsigned offset = 0;
    for (int p = 0; p < 4; p++) {
        c->count[p] = s->cached[p].count;
        c->start[p] = offset;
        unsigned pass_start = s->pass[p].count - c->count[p];
        for (unsigned j = 0; j < c->count[p]; j++) {
            uint32_t index = s->cached[p].v[j] - c->first + dest;
            s->pass[p].v[pass_start + j] = index;
            if (!s->failed)
                c->indices[offset + j] = index;
        }
        offset += c->count[p];
    }
    c->first = dest;
    c->valid = !s->failed;
    s->recording = 0;
}
