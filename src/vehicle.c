/* SPDX-License-Identifier: MIT */
#include "riptide/vehicle.h"
#include <math.h>
RiptideVehicleConfig riptide_vehicle_defaults(void) {
    return (RiptideVehicleConfig){4.4f, 1.8f, 1.5f, 1400,  38, 8,  12, 27,   8, .2f,
                             .7f,  .58f, 10,   1.65f, 14, 12, 9,  .36f, 1};
}
bool riptide_vehicle_config_valid(const RiptideVehicleConfig *c) {
    if (!c)
        return false;
    const float positive[] = {c->length,       c->width,          c->height,
                              c->mass,         c->max_speed,      c->reverse_speed,
                              c->acceleration, c->braking,        c->grip,
                              c->drag,         c->steering_angle, c->steering_response,
                              c->yaw_limit,    c->wheel_radius,   c->suspension_softness};
    for (size_t i = 0; i < sizeof positive / sizeof *positive; i++)
        if (!isfinite(positive[i]) || positive[i] <= 0 || positive[i] > 1e6f)
            return false;
    return c->steering_angle < 1.4f && isfinite(c->drift_grip) && c->drift_grip >= 0 &&
           c->drift_grip <= 1 && isfinite(c->drift_min_speed) && c->drift_min_speed >= 0 &&
           isfinite(c->boost_acceleration) && c->boost_acceleration >= 0 &&
           c->boost_acceleration <= 1e6f && isfinite(c->boost_speed) && c->boost_speed >= 0 &&
           c->boost_speed <= 1e6f;
}
void riptide_vehicle_reset(RiptideVehicle *v, RiptideVec3 position, float yaw) {
    if (v)
        *v = (RiptideVehicle){.pose = {.position = position, .yaw = yaw}, .grounded = true};
}
float riptide_vehicle_speed(const RiptideVehicle *v) {
    return v ? hypotf(v->velocity.x, v->velocity.z) : 0;
}
static float ground(RiptideHeightFn fn, void *user, float x, float z) {
    return fn ? fn(user, x, z) : 0;
}
static void spring(float *value, float *rate, float target, float frequency, float damping,
                   float dt, float limit) {
    int count = (int)ceilf(dt / .008f);
    float step = dt / count;
    for (int i = 0; i < count; i++) {
        *rate +=
            (frequency * frequency * (target - *value) - 2 * damping * frequency * (*rate)) * step;
        *value += *rate * step;
        if (fabsf(*value) > limit) {
            *value = copysignf(limit, *value);
            *rate *= .2f;
        }
    }
}
bool riptide_vehicle_step(RiptideVehicle *v, const RiptideVehicleConfig *c, RiptideControls input, RiptideHeightFn height,
                     void *user, float dt) {
    if (!v || !riptide_vehicle_config_valid(c) || !isfinite(dt) || dt <= 0 || dt > .05f ||
        !isfinite(input.throttle) || !isfinite(input.brake) || !isfinite(input.steering))
        return false;
    RiptideVec3 old = v->velocity;
    float throttle = riptide_clamp(input.throttle, -1, 1), brake = riptide_clamp(input.brake, 0, 1);
    float speed = riptide_vehicle_speed(v), sn = sinf(v->pose.yaw), cs = cosf(v->pose.yaw);
    float forward = v->velocity.x * sn + v->velocity.z * cs;
    bool drift = input.drift && fabsf(input.steering) > .15f && speed >= c->drift_min_speed;
    bool boost = input.boost && !drift && throttle > 0;
    v->steering +=
        (riptide_clamp(input.steering, -1, 1) - v->steering) * (1 - expf(-c->steering_response * dt));
    v->brake_light +=
        (brake > 0 || drift ? 1 - v->brake_light : -v->brake_light) * (1 - expf(-16 * dt));
    if (v->grounded) {
        float angle = v->steering * c->steering_angle / (1 + speed * .009f);
        float yaw = forward / fmaxf(1, c->length * .62f) * tanf(angle) * (drift ? 1.6f : 1);
        v->pose.yaw =
            riptide_angle_delta(v->pose.yaw + riptide_clamp(yaw, -c->yaw_limit, c->yaw_limit) * dt, 0);
        sn = sinf(v->pose.yaw);
        cs = cosf(v->pose.yaw);
        float side = v->velocity.x * cs - v->velocity.z * sn;
        float grip = 1 - expf(-c->grip * (drift ? c->drift_grip : 1) * dt);
        v->tiriptide_slip = fabsf(side);
        v->velocity.x -= cs * side * grip;
        v->velocity.z += sn * side * grip;
        float grade =
            (ground(height, user, v->pose.position.x + sn * 2, v->pose.position.z + cs * 2) -
             ground(height, user, v->pose.position.x - sn * 2, v->pose.position.z - cs * 2)) /
            4;
        float a = throttle * c->acceleration - 4 * grade + (boost ? c->boost_acceleration : 0);
        v->velocity.x += sn * a * dt;
        v->velocity.z += cs * a * dt;
        /* Braking reduces current motion without accidentally engaging reverse. */
        float current = riptide_vehicle_speed(v),
              factor = current > 0 ? fmaxf(0, 1 - brake * c->braking * dt / current) : 0;
        factor *= expf(-(fabsf(throttle) > .01f ? .16f : c->drag) * dt);
        v->velocity.x *= factor;
        v->velocity.z *= factor;
        current = riptide_vehicle_speed(v);
        float cap = forward < -.1f ? c->reverse_speed : c->max_speed + (boost ? c->boost_speed : 0);
        if (current > cap) {
            factor = fminf(current, fmaxf(cap, speed - 12 * dt)) / current;
            v->velocity.x *= factor;
            v->velocity.z *= factor;
        }
    } else {
        v->velocity.y -= 18 * dt;
        v->tiriptide_slip = 0;
        v->pose.yaw = riptide_angle_delta(v->pose.yaw + v->steering * .5f * dt, 0);
    }
    v->pose.position.x += v->velocity.x * dt;
    v->pose.position.z += v->velocity.z * dt;
    if (!v->grounded)
        v->pose.position.y += v->velocity.y * dt;
    float floor = ground(height, user, v->pose.position.x, v->pose.position.z);
    if (v->grounded || v->pose.position.y <= floor) {
        if (!v->grounded)
            v->heave_rate = -fminf(2, fabsf(v->velocity.y) * .15f);
        v->pose.position.y = floor;
        v->velocity.y = 0;
        v->grounded = true;
    }
    forward = v->velocity.x * sn + v->velocity.z * cs;
    v->wheel_spin = fmodf(v->wheel_spin + forward * dt / c->wheel_radius, 2 * RIPTIDE_PI);
    float ax = riptide_clamp((v->velocity.x - old.x) / dt, -60, 60);
    float az = riptide_clamp((v->velocity.z - old.z) / dt, -60, 60);
    float frequency = riptide_clamp(12.5f - logf(c->mass / 1400) * 2, 6, 17);
    float roll_limit = fminf(.09f, .24f / c->width), pitch_limit = fminf(.07f, .25f / c->length);
    float roll =
        riptide_clamp((ax * cs - az * sn) * .005f * c->suspension_softness, -roll_limit, roll_limit);
    float pitch =
        riptide_clamp((ax * sn + az * cs) * .004f * c->suspension_softness, -pitch_limit, pitch_limit);
    spring(&v->pose.roll, &v->roll_rate, roll, frequency, .72f, dt, roll_limit);
    spring(&v->pose.pitch, &v->pitch_rate, pitch, frequency * .9f, .75f, dt, pitch_limit);
    spring(&v->heave, &v->heave_rate, 0, frequency * 1.1f, .65f, dt, .15f);
    return true;
}
void riptide_vehicle_impulse(RiptideVehicle *v, const RiptideVehicleConfig *c, RiptideVec3 impulse) {
    if (!v || !c || c->mass <= 0)
        return;
    v->velocity.x += impulse.x / c->mass;
    v->velocity.y += impulse.y / c->mass;
    v->velocity.z += impulse.z / c->mass;
    if (v->velocity.y > 0)
        v->grounded = false;
}
float riptide_vehicle_collide(RiptideVehicle *a, const RiptideVehicleConfig *ca, RiptideVehicle *b,
                         const RiptideVehicleConfig *cb, float restitution) {
    if (!a || !b || a == b || !ca || !cb || ca->mass <= 0 || cb->mass <= 0)
        return 0;
    if (fabsf(a->pose.position.y - b->pose.position.y) > fmaxf(ca->height, cb->height))
        return 0;
    float dx = b->pose.position.x - a->pose.position.x,
          dz = b->pose.position.z - a->pose.position.z;
    float dist = hypotf(dx, dz), radius = (ca->width + cb->width) * .5f;
    if (dist >= radius)
        return 0;
    float nx = dist > 1e-6f ? dx / dist : 1, nz = dist > 1e-6f ? dz / dist : 0;
    float ia = 1 / ca->mass, ib = 1 / cb->mass, total = ia + ib, depth = radius - dist;
    a->pose.position.x -= nx * depth * ia / total;
    a->pose.position.z -= nz * depth * ia / total;
    b->pose.position.x += nx * depth * ib / total;
    b->pose.position.z += nz * depth * ib / total;
    float closing = (a->velocity.x - b->velocity.x) * nx + (a->velocity.z - b->velocity.z) * nz;
    if (closing <= 0)
        return 0;
    float impulse = closing * (1 + riptide_clamp(restitution, 0, 1)) / total;
    a->velocity.x -= nx * impulse * ia;
    a->velocity.z -= nz * impulse * ia;
    b->velocity.x += nx * impulse * ib;
    b->velocity.z += nz * impulse * ib;
    return closing;
}
float riptide_vehicle_collide_box(RiptideVehicle *v, const RiptideVehicleConfig *c, RiptideVec3 lo, RiptideVec3 hi,
                             float bounce) {
    if (!v || !c || lo.x > hi.x || lo.z > hi.z)
        return 0;
    float x = v->pose.position.x, z = v->pose.position.z, r = c->width * .5f;
    float dx = x - riptide_clamp(x, lo.x, hi.x), dz = z - riptide_clamp(z, lo.z, hi.z), d = hypotf(dx, dz);
    if (d >= r)
        return 0;
    float nx, nz, depth;
    if (d > 1e-6f) {
        nx = dx / d;
        nz = dz / d;
        depth = r - d;
    } else {
        float sides[] = {x - lo.x, hi.x - x, z - lo.z, hi.z - z};
        int side = 0;
        for (int i = 1; i < 4; i++)
            if (sides[i] < sides[side])
                side = i;
        nx = side == 0 ? -1 : side == 1 ? 1 : 0;
        nz = side == 2 ? -1 : side == 3 ? 1 : 0;
        depth = r + sides[side];
    }
    v->pose.position.x += nx * depth;
    v->pose.position.z += nz * depth;
    float closing = -(v->velocity.x * nx + v->velocity.z * nz);
    if (closing <= 0)
        return 0;
    float impulse = closing * (1 + riptide_clamp(bounce, 0, 1));
    v->velocity.x += nx * impulse;
    v->velocity.z += nz * impulse;
    return closing;
}
