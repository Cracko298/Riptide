/* SPDX-License-Identifier: MIT */
#include "riptide/world.h"
#include <math.h>
#include <stdlib.h>
static bool config_valid(const RiptideWorldConfig *c) {
    return c && isfinite(c->cell_size) && c->cell_size >= 8 && c->cell_size <= 4096 &&
           isfinite(c->relief) && c->relief >= 0 && c->relief <= 1000 && isfinite(c->water_level) &&
           fabsf(c->water_level) <= 1e6f && isfinite(c->water_threshold) &&
           c->water_threshold >= 0 && c->water_threshold <= 1 && c->artery_spacing >= 2 &&
           c->artery_spacing <= 64;
}
static bool coord_valid(int64_t x, int64_t z) {
    return x >= -RIPTIDE_CELL_LIMIT && x <= RIPTIDE_CELL_LIMIT && z >= -RIPTIDE_CELL_LIMIT && z <= RIPTIDE_CELL_LIMIT;
}
static int64_t floor_div(int64_t x, int n) {
    return x >= 0 ? x / n : -1 - (-(x + 1)) / n;
}
static float noise(uint32_t seed, int64_t x, int64_t z, int scale, uint32_t salt) {
    int64_t a = floor_div(x, scale), b = floor_div(z, scale);
    float u = (float)(x - a * scale) / scale, v = (float)(z - b * scale) / scale;
    u = u * u * (3 - 2 * u);
    v = v * v * (3 - 2 * v);
    float h[4];
    for (int i = 0; i < 4; i++)
        h[i] = (riptide_hash_cell(seed, a + (i & 1), b + (i >> 1), salt) & 65535) / 65535.f;
    return (h[0] + (h[1] - h[0]) * u) * (1 - v) + (h[2] + (h[3] - h[2]) * u) * v;
}
static bool wet(const RiptideWorldConfig *c, int64_t x, int64_t z) {
    float land = .76f * noise(c->seed, x, z, 56, 1961) + .24f * noise(c->seed, x, z, 19, 1962);
    return land < c->water_threshold;
}
static float node_height(const RiptideWorldConfig *c, int64_t x, int64_t z) {
    /* Shared lattice vertices make adjacent chunks agree at their seams. */
    float coast = 1;
    for (int j = -2; j <= 1; j++)
        for (int i = -2; i <= 1; i++)
            if (wet(c, x + i, z + j))
                coast = fminf(coast, riptide_clamp(fmaxf(fabsf(i + .5f), fabsf(j + .5f)) - .5f, 0, 1));
    return c->water_level +
           coast * c->relief *
               (.75f * noise(c->seed, x, z, 10, 1981) + .25f * noise(c->seed, x, z, 5, 1982));
}
RiptideWorldConfig riptide_world_defaults(uint32_t seed) {
    return (RiptideWorldConfig){seed, 64, 12, 0, .43f, 4};
}
float riptide_world_height(const RiptideWorldConfig *c, double x, double z) {
    if (!config_valid(c) || !isfinite(x) || !isfinite(z))
        return 0;
    double gx = x / c->cell_size, gz = z / c->cell_size;
    if (gx < -RIPTIDE_CELL_LIMIT || gx >= RIPTIDE_CELL_LIMIT || gz < -RIPTIDE_CELL_LIMIT || gz >= RIPTIDE_CELL_LIMIT)
        return 0;
    int64_t ix = (int64_t)floor(gx), iz = (int64_t)floor(gz);
    float u = (float)(gx - ix), v = (float)(gz - iz);
    float a = node_height(c, ix, iz), b = node_height(c, ix + 1, iz);
    float d = node_height(c, ix, iz + 1), e = node_height(c, ix + 1, iz + 1);
    return (a + (b - a) * u) * (1 - v) + (d + (e - d) * u) * v;
}
bool riptide_world_cell(const RiptideWorldConfig *c, int64_t x, int64_t z, RiptideCell *cell) {
    if (!cell || !config_valid(c) || !coord_valid(x, z))
        return false;
    RiptideCell result = {.x = x,
                     .z = z,
                     .variant = riptide_hash_cell(c->seed, x, z, 194),
                     .surface = wet(c, x, z) ? RIPTIDE_WATER : RIPTIDE_LAND};
    result.height = riptide_world_height(c, (x + .5) * c->cell_size, (z + .5) * c->cell_size);
    static const int dx[] = {0, 1, 0, -1}, dz[] = {1, 0, -1, 0};
    if (result.surface == RIPTIDE_LAND)
        for (int d = 0; d < 4; d++) {
            int64_t nx = x + dx[d], nz = z + dz[d];
            if (wet(c, nx, nz))
                continue;
            /* Canonical edge keys ensure both cells choose the same connection. */
            int64_t ex = x < nx ? x : nx, ez = z < nz ? z : nz;
            bool artery =
                (d & 1) ? z % (int)c->artery_spacing == 0 : x % (int)c->artery_spacing == 0;
            if (artery || riptide_hash_cell(c->seed, ex, ez, 2100 + (d & 1)) % 3 == 0)
                result.roads |= 1u << d;
        }
    *cell = result;
    return true;
}
bool riptide_world_rebase(RiptideOrigin *o, RiptideOrigin next, float size, RiptideVec3 *p, size_t n) {
    if (!o || (!p && n) || !coord_valid(o->x, o->z) || !coord_valid(next.x, next.z) ||
        !isfinite(size) || size <= 0 || size > 4096)
        return false;
    double dx = (double)(next.x - o->x) * size, dz = (double)(next.z - o->z) * size;
    for (size_t i = 0; i < n; i++) {
        p[i].x = (float)(p[i].x - dx);
        p[i].z = (float)(p[i].z - dz);
    }
    *o = next;
    return true;
}
typedef struct {
    RiptideCell cell;
    bool active;
} Slot;
struct RiptideStream {
    RiptideWorldConfig config;
    unsigned radius, side, capacity, pending;
    RiptideCellEventFn event;
    void *user;
    Slot *slots;
};
RiptideStream *riptide_stream_create(RiptideWorldConfig c, unsigned radius, RiptideCellEventFn fn, void *user) {
    if (!config_valid(&c) || radius > 64)
        return NULL;
    RiptideStream *s = calloc(1, sizeof *s);
    if (!s)
        return NULL;
    s->side = 2 * radius + 1;
    s->capacity = s->side * s->side;
    s->radius = radius;
    s->slots = calloc(s->capacity, sizeof *s->slots);
    if (!s->slots) {
        free(s);
        return NULL;
    }
    s->config = c;
    s->event = fn;
    s->user = user;
    s->pending = s->capacity;
    return s;
}
static unsigned mod(int64_t n, unsigned d) {
    int64_t r = n % d;
    return (unsigned)(r < 0 ? r + d : r);
}
unsigned riptide_stream_update(RiptideStream *s, int64_t x, int64_t z, unsigned budget) {
    if (!s || !coord_valid(x, z) || x < -RIPTIDE_CELL_LIMIT + s->radius ||
        x > RIPTIDE_CELL_LIMIT - s->radius || z < -RIPTIDE_CELL_LIMIT + s->radius ||
        z > RIPTIDE_CELL_LIMIT - s->radius)
        return 0;
    for (unsigned i = 0; i < s->capacity; i++) {
        Slot *slot = &s->slots[i];
        if (slot->active &&
            (llabs(slot->cell.x - x) > s->radius || llabs(slot->cell.z - z) > s->radius)) {
            if (s->event)
                s->event(s->user, &slot->cell, false);
            slot->active = false;
        }
    }
    unsigned entered = 0;
    s->pending = 0;
    /* Ring order prioritizes playable cells before distant scenery. */
    for (int r = 0; r <= (int)s->radius; r++)
        for (int j = -r; j <= r; j++)
            for (int i = -r; i <= r; i++) {
                if (abs(i) != r && abs(j) != r)
                    continue;
                int64_t cx = x + i, cz = z + j;
                Slot *slot = &s->slots[mod(cz, s->side) * s->side + mod(cx, s->side)];
                if (slot->active)
                    continue;
                if (entered >= budget) {
                    s->pending++;
                    continue;
                }
                if (riptide_world_cell(&s->config, cx, cz, &slot->cell)) {
                    slot->active = true;
                    entered++;
                    if (s->event)
                        s->event(s->user, &slot->cell, true);
                }
            }
    return entered;
}
unsigned riptide_stream_pending(const RiptideStream *s) {
    return s ? s->pending : 0;
}
void riptide_stream_destroy(RiptideStream *s) {
    if (!s)
        return;
    for (unsigned i = 0; i < s->capacity; i++)
        if (s->slots[i].active && s->event)
            s->event(s->user, &s->slots[i].cell, false);
    free(s->slots);
    free(s);
}
