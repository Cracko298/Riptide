/* SPDX-License-Identifier: MIT */
#include "riptide/physics.h"
#include <math.h>
void riptide_body_step(RiptideBody *b, RiptideVec3 g, float damping, float bounce, RiptideHeightFn height, void *user,
                  float dt) {
    if (!b || b->inverse_mass <= 0 || !isfinite(dt) || dt <= 0 || dt > .05f)
        return;
    float drag = expf(-fmaxf(0, damping) * dt);
    b->velocity.x = (b->velocity.x + g.x * dt) * drag;
    b->velocity.y = (b->velocity.y + g.y * dt) * drag;
    b->velocity.z = (b->velocity.z + g.z * dt) * drag;
    b->position.x += b->velocity.x * dt;
    b->position.y += b->velocity.y * dt;
    b->position.z += b->velocity.z * dt;
    float floor = (height ? height(user, b->position.x, b->position.z) : 0) + fmaxf(0, b->radius);
    if (b->position.y < floor) {
        b->position.y = floor;
        if (b->velocity.y < 0)
            b->velocity.y = -b->velocity.y * riptide_clamp(bounce, 0, 1);
        if (b->velocity.y < .2f)
            b->velocity.y = 0;
    }
}
float riptide_distance_solve(RiptideBody *a, RiptideBody *b, float length, float stiffness, bool rope) {
    if (!a || !b || a == b || length < 0)
        return 0;
    float ia = fmaxf(0, a->inverse_mass), ib = fmaxf(0, b->inverse_mass), sum = ia + ib;
    if (sum <= 0)
        return 0;
    RiptideVec3 d = {b->position.x - a->position.x, b->position.y - a->position.y,
                b->position.z - a->position.z};
    float dist = sqrtf(d.x * d.x + d.y * d.y + d.z * d.z), error = dist - length;
    if (rope && error <= 0)
        return 0;
    RiptideVec3 n = dist > 1e-6f ? (RiptideVec3){d.x / dist, d.y / dist, d.z / dist} : (RiptideVec3){1, 0, 0};
    float k = riptide_clamp(stiffness, 0, 1), fix = error * k / sum;
    a->position.x += n.x * fix * ia;
    a->position.y += n.y * fix * ia;
    a->position.z += n.z * fix * ia;
    b->position.x -= n.x * fix * ib;
    b->position.y -= n.y * fix * ib;
    b->position.z -= n.z * fix * ib;
    float speed = (b->velocity.x - a->velocity.x) * n.x + (b->velocity.y - a->velocity.y) * n.y +
                  (b->velocity.z - a->velocity.z) * n.z;
    if (!rope || speed > 0) {
        float impulse = speed * k / sum;
        a->velocity.x += n.x * impulse * ia;
        a->velocity.y += n.y * impulse * ia;
        a->velocity.z += n.z * impulse * ia;
        b->velocity.x -= n.x * impulse * ib;
        b->velocity.y -= n.y * impulse * ib;
        b->velocity.z -= n.z * impulse * ib;
    }
    return fabsf(error);
}
