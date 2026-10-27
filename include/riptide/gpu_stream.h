/* SPDX-License-Identifier: MIT */
#ifndef RIPTIDE_GPU_STREAM_H
#define RIPTIDE_GPU_STREAM_H
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
/* One instance describes a quad, triangle, or a screen-width neon ribbon.
   p.w == -1 denotes world coordinates; other values are homogeneous clip w. */
typedef struct {
    float p[4][4], color[4][4], meta[4];
} RiptideGpuPrimitive;
#ifdef __cplusplus
static_assert(sizeof(RiptideGpuPrimitive) == 144, "GPU primitive stride");
#else
_Static_assert(sizeof(RiptideGpuPrimitive) == 144, "GPU primitive stride");
#endif
typedef struct {
    uint32_t *v;
    unsigned count, capacity;
} RiptideGpuIndices;
typedef struct {
    uint64_t key;
    unsigned start[4], count[4];
    unsigned first, size, used;
    uint32_t *indices;
    unsigned capacity;
    bool valid;
} RiptideGpuCache;
typedef struct {
    unsigned first, size;
} RiptideGpuSpan;
#define GPU_CACHE_SLOTS 2048
#define GPU_CACHE_WAYS 2
#define GPU_CACHE_ENTRIES (GPU_CACHE_SLOTS * GPU_CACHE_WAYS)
#define GPU_DYNAMIC_BIT 0x80000000u
typedef struct RiptideGpuStream {
    RiptideGpuPrimitive *dynamic, *fixed;
    unsigned dynamic_count, dynamic_capacity, fixed_count, fixed_capacity;
    RiptideGpuIndices pass[4], cached[4];
    RiptideGpuCache *cache;
    RiptideGpuSpan *free_spans, *retired, *dirty;
    unsigned free_count, free_capacity, retired_count, retired_capacity, dirty_count,
        dirty_capacity;
    unsigned sweep;
    size_t index_bytes;
    uint64_t scene_key, serial;
    unsigned revision, hits, misses, recording, dropped;
    unsigned epoch, stale, compactions, invalidations;
    bool failed, cache_disabled;
} RiptideGpuStream;
#ifdef __cplusplus
extern "C" {
#endif
/** Allocate an optional cached command stream for a custom GPU backend.
 * @return Owned stream or NULL. Creation must be serialized across threads.
 * This stores geometry only; it does not create a graphics device. */
RiptideGpuStream *riptide_gpu_stream_new(void);
/** Measure allocated CPU storage in constant time.
 * @param stream Stream to inspect; NULL is accepted.
 * @return Estimated allocated bytes, excluding allocator overhead. */
size_t riptide_gpu_stream_bytes(const RiptideGpuStream *stream);
/** Choose a rounded growth capacity for a backend buffer.
 * @param current Existing element capacity.
 * @param needed Minimum requested element count.
 * @return New capacity, or zero on unsigned overflow. */
unsigned riptide_gpu_stream_capacity(unsigned current, unsigned needed);
/** Destroy a stream and all cached geometry.
 * @param stream Owned stream; NULL is accepted. */
void riptide_gpu_stream_free(RiptideGpuStream *stream);
/** Start a new command frame, retaining reusable fixed geometry.
 * @param stream Stream with no unfinished cache recording.
 * Previous CPU command lists must already be consumed. GPU backends must fence
 * in-flight uploads before reusing retired fixed-buffer ranges. */
void riptide_gpu_stream_reset(RiptideGpuStream *stream);
/** Invalidate geometry when world content changes.
 * @param stream Mutable stream.
 * @param key Application-chosen scene/content revision.
 * A different key discards the current frame. Call before emitting geometry. */
void riptide_gpu_stream_scene(RiptideGpuStream *stream, uint64_t key);
/** Append a triangle/quad/ribbon command to a pass.
 * @param stream Mutable stream, used by one thread at a time.
 * @param pass Pass index in [0, 3].
 * @param primitive Command copied by value; see docs/GPU.md for encoding.
 * Allocation failures set failed and increment dropped; recreate a failed stream. */
void riptide_gpu_stream_emit(RiptideGpuStream *stream, int pass, const RiptideGpuPrimitive *primitive);
/** Replay cached geometry or begin recording a cache miss.
 * @param stream Mutable stream, with no recording already open.
 * @param slot Spatial cache slot in [0, GPU_CACHE_SLOTS).
 * @param key Complete geometry key, including content and any baked view state.
 * @return True on replay. Otherwise emit geometry, then call riptide_gpu_cache_end.
 * Check stream->failed separately; false can also indicate an invalid operation. */
bool riptide_gpu_cache_begin(RiptideGpuStream *stream, unsigned slot, uint64_t key);
/** Finish a cache recording and publish its dirty upload ranges.
 * @param stream Stream with a recording, or NULL. No-op if no recording is open.
 * Do not nest recordings or reset/change scenes while recording. */
void riptide_gpu_cache_end(RiptideGpuStream *stream);
#ifdef __cplusplus
}
#endif
#endif
