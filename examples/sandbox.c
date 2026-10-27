/* SPDX-License-Identifier: MIT */
#include "sandbox.h"
#include <math.h>
#include <string.h>
static float terrain(void *context, float x, float z) {
    Sandbox *s = context;
    return riptide_world_height(&s->world, x + (double)s->origin.x * s->world.cell_size,
                           z + (double)s->origin.z * s->world.cell_size);
}
static void cell_event(void *context, const RiptideCell *cell, bool entering) {
    Sandbox *s = context;
    for (unsigned i = 0; i < SANDBOX_CELLS; i++) {
        if (entering && !s->resident[i]) {
            s->cells[i] = *cell;
            s->resident[i] = true;
            return;
        }
        if (!entering && s->resident[i] && s->cells[i].x == cell->x && s->cells[i].z == cell->z) {
            s->resident[i] = false;
            return;
        }
    }
}
bool sandbox_init(Sandbox *s, unsigned w, unsigned h) {
    memset(s, 0, sizeof *s);
    s->width = w;
    s->height = h;
    s->world = riptide_world_defaults(18271);
    s->world.water_threshold = 0;
    s->world.relief = 7;
    s->world.artery_spacing = 2;
    s->handling = riptide_vehicle_defaults();
    s->renderer = riptide_render_create(160000);
    s->canvas = riptide_canvas_create(w, h);
    s->stream = riptide_stream_create(s->world, 4, cell_event, s);
    if (!s->renderer || !s->canvas || !s->stream) {
        sandbox_free(s);
        return false;
    }
    riptide_stream_update(s->stream, 0, 0, SANDBOX_CELLS);
    riptide_vehicle_reset(&s->player, (RiptideVec3){32, terrain(s, 32, 12), 12}, 0);
    s->previous = s->player.pose;
    for (int i = 0; i < SANDBOX_TRAFFIC; i++) {
        float x = 32 + (i % 4) * 128 + (i & 1 ? -3.5f : 3.5f), z = -100 + (i / 4) * 80;
        riptide_vehicle_reset(&s->traffic[i], (RiptideVec3){x, terrain(s, x, z), z}, i & 1 ? RIPTIDE_PI : 0);
    }
    for (int i = 0; i < 5; i++)
        s->chain[i] = (RiptideBody){.position = {32, terrain(s, 32, 12) + 1, 8 - i * 2.f},
                               .inverse_mass = i ? 1.f / 40 : 0,
                               .radius = .3f};
    return true;
}
static void maybe_rebase(Sandbox *s) {
    RiptideVec3 p = s->player.pose.position;
    if (fabsf(p.x) < 1024 && fabsf(p.z) < 1024)
        return;
    RiptideOrigin next = {s->origin.x + (int64_t)floorf(p.x / s->world.cell_size),
                     s->origin.z + (int64_t)floorf(p.z / s->world.cell_size)};
    RiptideVec3 positions[2 + SANDBOX_TRAFFIC + 5];
    positions[0] = p;
    positions[1] = s->previous.position;
    for (int i = 0; i < SANDBOX_TRAFFIC; i++)
        positions[2 + i] = s->traffic[i].pose.position;
    for (int i = 0; i < 5; i++)
        positions[2 + SANDBOX_TRAFFIC + i] = s->chain[i].position;
    riptide_world_rebase(&s->origin, next, s->world.cell_size, positions, 2 + SANDBOX_TRAFFIC + 5);
    s->player.pose.position = positions[0];
    s->previous.position = positions[1];
    for (int i = 0; i < SANDBOX_TRAFFIC; i++)
        s->traffic[i].pose.position = positions[2 + i];
    for (int i = 0; i < 5; i++)
        s->chain[i].position = positions[2 + SANDBOX_TRAFFIC + i];
}
static RiptideVec3 building_position(const Sandbox *s, const RiptideCell *c) {
    return (RiptideVec3){(float)(c->x - s->origin.x) * 64 + 8, 0, (float)(c->z - s->origin.z) * 64 + 8};
}
void sandbox_step(Sandbox *s, RiptideControls input, float dt) {
    s->previous = s->player.pose;
    s->time += dt;
    riptide_vehicle_step(&s->player, &s->handling, input, terrain, s, dt);
    RiptideVehicleConfig npc = s->handling;
    npc.max_speed = 13;
    for (int i = 0; i < SANDBOX_TRAFFIC; i++) {
        RiptideVehicle *v = &s->traffic[i];
        riptide_vehicle_step(v, &npc, (RiptideControls){.throttle = .6f}, terrain, s, dt);
        if (hypotf(v->pose.position.x - s->player.pose.position.x,
                   v->pose.position.z - s->player.pose.position.z) > 350) {
            double gx = s->player.pose.position.x + (double)s->origin.x * 64;
            int64_t lane = (int64_t)floor(gx / 128);
            float x = (float)(lane * 128 - s->origin.x * 64) + 32 + (i % 3 - 1) * 128 +
                      (i & 1 ? -3.5f : 3.5f);
            float z = s->player.pose.position.z + (i & 1 ? 220 : -220);
            riptide_vehicle_reset(v, (RiptideVec3){x, terrain(s, x, z), z}, i & 1 ? RIPTIDE_PI : 0);
        }
        if (riptide_vehicle_collide(&s->player, &s->handling, v, &npc, .25f) > 2)
            s->collisions++;
    }
    for (int i = 0; i < SANDBOX_CELLS; i++)
        if (s->resident[i]) {
            RiptideVec3 p = building_position(s, &s->cells[i]);
            if (riptide_vehicle_collide_box(&s->player, &s->handling, p, (RiptideVec3){p.x + 12, 0, p.z + 12},
                                       .25f) > 2)
                s->collisions++;
        }
    s->chain[0].position = s->player.pose.position;
    s->chain[0].position.y += 1;
    s->chain[0].velocity = s->player.velocity;
    for (int i = 1; i < 5; i++)
        riptide_body_step(&s->chain[i], (RiptideVec3){0, -18, 0}, .7f, .25f, terrain, s, dt);
    for (int iteration = 0; iteration < 8; iteration++)
        for (int i = 1; i < 5; i++)
            riptide_distance_solve(&s->chain[i - 1], &s->chain[i], 2, .75f, false);
    maybe_rebase(s);
    int64_t cx = s->origin.x + (int64_t)floorf(s->player.pose.position.x / 64);
    int64_t cz = s->origin.z + (int64_t)floorf(s->player.pose.position.z / 64);
    riptide_stream_update(s->stream, cx, cz, 12);
}
static RiptideVec3 ground_point(Sandbox *s, float x, float z, float lift) {
    return (RiptideVec3){x, terrain(s, x, z) + lift, z};
}
static void patch(Sandbox *s, float x, float z, float w, float d, float lift, RiptideColor color) {
    /* Align both road and ground subdivision to limit heightfield intersections. */
    int nx = (int)ceilf(w / 8), nz = (int)ceilf(d / 8);
    for (int j = 0; j < nz; j++)
        for (int i = 0; i < nx; i++) {
            float a = x + w * i / nx, b = x + w * (i + 1) / nx, c = z + d * j / nz,
                  e = z + d * (j + 1) / nz;
            riptide_render_quad(s->renderer, ground_point(s, a, c, lift), ground_point(s, b, c, lift),
                           ground_point(s, b, e, lift), ground_point(s, a, e, lift), color);
        }
}
static void car(Sandbox *s, const RiptideVehicle *v, RiptidePose pose, RiptideColor color) {
    pose.position.y += .65f + v->heave;
    riptide_render_box(s->renderer, pose, (RiptideVec3){1.8f, .65f, 4.4f}, color,
                  (RiptideColor){.3f, .5f, .65f, 1}, false);
    pose.position.y += .55f;
    riptide_render_box(s->renderer, pose, (RiptideVec3){1.5f, .65f, 2.2f}, (RiptideColor){.08f, .13f, .18f, 1},
                  color, true);
    float sn = sinf(pose.yaw), cs = cosf(pose.yaw);
    for (int i = 0; i < 4; i++) {
        float x = i & 1 ? .95f : -.95f, z = i & 2 ? 1.35f : -1.35f;
        RiptidePose wheel = {.position = {pose.position.x + x * cs + z * sn, v->pose.position.y + .38f,
                                     pose.position.z + z * cs - x * sn},
                        .yaw = pose.yaw,
                        .pitch = v->wheel_spin};
        riptide_render_box(s->renderer, wheel, (RiptideVec3){.3f, .6f, .6f}, (RiptideColor){.025f, .03f, .04f, 1},
                      (RiptideColor){.3f, .35f, .4f, 1}, false);
    }
}
bool sandbox_draw(Sandbox *s, float alpha) {
    RiptidePose pose = riptide_pose_lerp(s->previous, s->player.pose, alpha);
    float sn = sinf(pose.yaw), cs = cosf(pose.yaw);
    RiptideCamera camera = {
        .eye = {pose.position.x - sn * 14, pose.position.y + 8, pose.position.z - cs * 14},
        .target = {pose.position.x + sn * 7, pose.position.y + 1, pose.position.z + cs * 7},
        .vertical_fov = .95f,
        .near_clip = .25f,
        .far_clip = 340,
        .fog = {.07f, .1f, .15f, 1},
        .fog_enabled = true,
        .bloom_layers = 2};
    if (!riptide_render_begin(s->renderer, s->width, s->height, camera))
        return false;
    for (int i = 0; i < SANDBOX_CELLS; i++)
        if (s->resident[i]) {
            RiptideCell *c = &s->cells[i];
            float x = (float)(c->x - s->origin.x) * 64, z = (float)(c->z - s->origin.z) * 64;
            if (hypotf(x + 32 - pose.position.x, z + 32 - pose.position.z) > 260)
                continue;
            patch(s, x, z, 64, 64, 0, (RiptideColor){.09f, .14f, .12f, 1});
            if (c->roads)
                patch(s, x + 23, z + 23, 18, 18, .10f, (RiptideColor){.065f, .075f, .09f, 1});
            if (c->roads & 1)
                patch(s, x + 23, z + 32, 18, 32, .10f, (RiptideColor){.065f, .075f, .09f, 1});
            if (c->roads & 2)
                patch(s, x + 32, z + 23, 32, 18, .10f, (RiptideColor){.065f, .075f, .09f, 1});
            if (c->roads & 4)
                patch(s, x + 23, z, 18, 32, .10f, (RiptideColor){.065f, .075f, .09f, 1});
            if (c->roads & 8)
                patch(s, x, z + 23, 32, 18, .10f, (RiptideColor){.065f, .075f, .09f, 1});
            for (int k = 0; k < 4; k++) {
                if (c->roads & 5)
                    riptide_render_line(s->renderer, ground_point(s, x + 32, z + k * 16, .14f),
                                   ground_point(s, x + 32, z + k * 16 + 7, .14f),
                                   (RiptideColor){.7f, .65f, .4f, 1}, 1, false);
                if (c->roads & 10)
                    riptide_render_line(s->renderer, ground_point(s, x + k * 16, z + 32, .14f),
                                   ground_point(s, x + k * 16 + 7, z + 32, .14f),
                                   (RiptideColor){.7f, .65f, .4f, 1}, 1, false);
            }
            RiptideVec3 b = building_position(s, c);
            float height = 5 + (c->variant % 7) * 2.f;
            b.x += 6;
            b.z += 6;
            b.y = terrain(s, b.x, b.z) + height * .5f;
            RiptideColor accent =
                (c->variant & 1) ? (RiptideColor){.28f, .65f, .7f, 1} : (RiptideColor){.7f, .45f, .25f, 1};
            riptide_render_box(s->renderer, (RiptidePose){.position = b}, (RiptideVec3){12, height, 12},
                          (RiptideColor){.14f, .17f, .2f, 1}, accent, false);
            for (int f = 0; f < (int)height / 3; f++) {
                RiptideVec3 a = {b.x - 4, b.y - height * .5f + 2 + f * 3, b.z + 6.02f};
                riptide_render_line(s->renderer, a, (RiptideVec3){a.x + 8, a.y, a.z}, accent, 1.5f, true);
            }
        }
    car(s, &s->player, pose, (RiptideColor){.9f, .38f, .12f, 1});
    for (int i = 0; i < SANDBOX_TRAFFIC; i++)
        car(s, &s->traffic[i], s->traffic[i].pose, (RiptideColor){.16f, .4f, .55f, 1});
    for (int i = 1; i < 5; i++) {
        riptide_render_line(s->renderer, s->chain[i - 1].position, s->chain[i].position,
                       (RiptideColor){.6f, .65f, .7f, 1}, 1.5f, false);
        riptide_render_box(s->renderer, (RiptidePose){.position = s->chain[i].position},
                      (RiptideVec3){.6f, .6f, .6f}, (RiptideColor){.5f, .3f, .1f, 1},
                      (RiptideColor){.8f, .65f, .25f, 1}, false);
    }
    riptide_render_rect(s->renderer, 16, s->height - 26, 180, 10, (RiptideColor){.1f, .13f, .16f, .8f});
    riptide_render_rect(s->renderer, 16, s->height - 26,
                   180 * riptide_clamp(riptide_vehicle_speed(&s->player) / 47, 0, 1), 10,
                   (RiptideColor){.2f, .8f, .6f, 1});
    return riptide_canvas_draw(s->canvas, s->renderer, camera.fog);
}
void sandbox_free(Sandbox *s) {
    riptide_stream_destroy(s->stream);
    riptide_render_destroy(s->renderer);
    riptide_canvas_destroy(s->canvas);
    s->stream = NULL;
    s->renderer = NULL;
    s->canvas = NULL;
}
