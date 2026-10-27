/* SPDX-License-Identifier: MIT */
#include "riptide/core.h"
#include <math.h>
float riptide_clamp(float value, float lo, float hi) {
    return value < lo ? lo : value > hi ? hi : value;
}
float riptide_angle_delta(float a, float b) {
    float d = fmodf(a - b + RIPTIDE_PI, 2 * RIPTIDE_PI);
    return (d < 0 ? d + 2 * RIPTIDE_PI : d) - RIPTIDE_PI;
}
uint32_t riptide_hash_cell(uint32_t seed, int64_t x, int64_t z, uint32_t salt) {
    uint64_t a = (uint64_t)x, b = (uint64_t)z;
    uint32_t h = (uint32_t)a * 0x8da6b343u ^ (uint32_t)b * 0xd8163841u ^
                 (seed + salt + (uint32_t)(a >> 32) * 71u + (uint32_t)(b >> 32) * 193u);
    h ^= h >> 16;
    h *= 0x7feb352du;
    h ^= h >> 15;
    h *= 0x846ca68bu;
    return h ^ (h >> 16);
}
bool riptide_clock_init(RiptideClock *c, double hz, unsigned max_steps) {
    if (!c || !isfinite(hz) || hz < 1 || hz > 1000 || !max_steps || max_steps > 1000)
        return false;
    *c = (RiptideClock){.step = 1 / hz, .max_steps = max_steps};
    return true;
}
unsigned riptide_clock_advance(RiptideClock *c, double elapsed) {
    if (!c || c->step <= 0 || !isfinite(elapsed) || elapsed < 0)
        return 0;
    double limit = c->step * c->max_steps;
    if (elapsed > limit) {
        c->dropped_seconds += elapsed - limit;
        elapsed = limit;
    }
    c->accumulator += elapsed;
    unsigned n = (unsigned)floor((c->accumulator + c->step * 1e-9) / c->step);
    if (n > c->max_steps)
        n = c->max_steps;
    c->accumulator = fmax(0, c->accumulator - n * c->step);
    return n;
}
float riptide_clock_alpha(const RiptideClock *c) {
    return c && c->step > 0 ? riptide_clamp((float)(c->accumulator / c->step), 0, .99999994f) : 0;
}
RiptidePose riptide_pose_lerp(RiptidePose a, RiptidePose b, float t) {
    t = riptide_clamp(t, 0, 1);
    return (RiptidePose){{a.position.x + (b.position.x - a.position.x) * t,
                     a.position.y + (b.position.y - a.position.y) * t,
                     a.position.z + (b.position.z - a.position.z) * t},
                    a.yaw + riptide_angle_delta(b.yaw, a.yaw) * t,
                    a.pitch + riptide_angle_delta(b.pitch, a.pitch) * t,
                    a.roll + riptide_angle_delta(b.roll, a.roll) * t};
}
