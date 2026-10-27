/* SPDX-License-Identifier: MIT */
#include "riptide/riptide.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
static unsigned checks;
#define CHECK(expr)                                                                                \
    do {                                                                                           \
        checks++;                                                                                  \
        if (!(expr)) {                                                                             \
            fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #expr);                             \
            exit(1);                                                                               \
        }                                                                                          \
    } while (0)
static bool near(float a, float b, float tolerance) {
    return fabsf(a - b) <= tolerance;
}
static void test_clock(void) {
    RiptideClock c;
    CHECK(riptide_clock_init(&c, 120, 12));
    unsigned steps = 0;
    for (int i = 0; i < 60; i++)
        steps += riptide_clock_advance(&c, 1. / 60);
    CHECK(steps == 120);
    CHECK(riptide_clock_alpha(&c) < .001f);
    CHECK(riptide_clock_advance(&c, 1) == 12);
    CHECK(c.dropped_seconds > .89);
    CHECK(riptide_clock_advance(&c, NAN) == 0);
    CHECK(riptide_clock_advance(&c, -1) == 0);
    CHECK(!riptide_clock_init(&c, 0, 1));
    RiptidePose a = {.yaw = RIPTIDE_PI - .1f}, b = {.yaw = -RIPTIDE_PI + .1f};
    CHECK(fabsf(riptide_angle_delta(riptide_pose_lerp(a, b, .5f).yaw, RIPTIDE_PI)) < .001f);
}
static float hill(void *ctx, float x, float z) {
    (void)ctx;
    (void)x;
    return z * .05f;
}
static void test_vehicle(void) {
    RiptideVehicleConfig c = riptide_vehicle_defaults();
    CHECK(riptide_vehicle_config_valid(&c));
    RiptideVehicle a, b;
    riptide_vehicle_reset(&a, (RiptideVec3){0}, 0);
    b = a;
    for (int i = 0; i < 600; i++)
        CHECK(riptide_vehicle_step(&a, &c, (RiptideControls){.throttle = 1}, NULL, NULL, 1.f / 60));
    for (int i = 0; i < 1200; i++)
        CHECK(riptide_vehicle_step(&b, &c, (RiptideControls){.throttle = 1}, NULL, NULL, 1.f / 120));
    CHECK(near(a.pose.position.z, b.pose.position.z, 1));
    CHECK(riptide_vehicle_speed(&a) <= c.max_speed + .01f);
    CHECK(a.pose.position.z > 200);
    CHECK(fabsf(a.pose.position.x) < .001f);
    for (int i = 0; i < 300; i++)
        riptide_vehicle_step(&a, &c, (RiptideControls){.brake = 1}, NULL, NULL, 1.f / 60);
    CHECK(riptide_vehicle_speed(&a) < .001f);
    CHECK(a.velocity.z >= 0);
    for (int i = 0; i < 180; i++)
        riptide_vehicle_step(&a, &c, (RiptideControls){.throttle = -1}, NULL, NULL, 1.f / 60);
    CHECK(a.velocity.z < 0);
    CHECK(riptide_vehicle_speed(&a) <= c.reverse_speed + .1f);
    RiptideVehicle before = a;
    CHECK(!riptide_vehicle_step(&a, &c, (RiptideControls){0}, NULL, NULL, .5f));
    CHECK(memcmp(&a, &before, sizeof a) == 0);
    c.grip = NAN;
    CHECK(!riptide_vehicle_config_valid(&c));
    c = riptide_vehicle_defaults();
    riptide_vehicle_reset(&a, (RiptideVec3){0}, 0);
    for (int i = 0; i < 120; i++)
        riptide_vehicle_step(&a, &c, (RiptideControls){.throttle = 1, .steering = .25f}, hill, NULL,
                        1.f / 120);
    CHECK(a.pose.yaw > 0);
    CHECK(a.pose.position.x > 0);
    CHECK(near(a.pose.position.y, a.pose.position.z * .05f, .001f));
    riptide_vehicle_impulse(&a, &c, (RiptideVec3){0, c.mass * 7, 0});
    CHECK(!a.grounded);
    float highest = a.pose.position.y;
    for (int i = 0; i < 300; i++) {
        riptide_vehicle_step(&a, &c, (RiptideControls){0}, hill, NULL, 1.f / 120);
        highest = fmaxf(highest, a.pose.position.y);
    }
    CHECK(highest > 1);
    CHECK(a.grounded);
    CHECK(isfinite(a.heave));
    riptide_vehicle_reset(&a, (RiptideVec3){0}, 0);
    riptide_vehicle_reset(&b, (RiptideVec3){1, 0, 0}, 0);
    a.velocity.x = 10;
    RiptideVehicleConfig heavy = c;
    heavy.mass *= 2;
    float momentum = a.velocity.x * c.mass + b.velocity.x * heavy.mass;
    CHECK(riptide_vehicle_collide(&a, &c, &b, &heavy, 1) > 9.9f);
    CHECK(near(a.velocity.x * c.mass + b.velocity.x * heavy.mass, momentum, .1f));
    CHECK(near(b.pose.position.x - a.pose.position.x, c.width, .001f));
    riptide_vehicle_reset(&a, (RiptideVec3){.5f, 0, .5f}, 0);
    a.velocity.x = 10;
    riptide_vehicle_collide_box(&a, &c, (RiptideVec3){0}, (RiptideVec3){1, 0, 1}, .5f);
    CHECK(a.pose.position.x < 0 || a.pose.position.x > 1 || a.pose.position.z < 0 ||
          a.pose.position.z > 1);
}
typedef struct {
    int enters, leaves, live;
    RiptideCell cells[25];
    bool used[25];
} Events;
static void event(void *context, const RiptideCell *cell, bool entering) {
    Events *e = context;
    if (entering) {
        for (int i = 0; i < 25; i++)
            CHECK(!(e->used[i] && e->cells[i].x == cell->x && e->cells[i].z == cell->z));
        int at = 0;
        while (at < 25 && e->used[at])
            at++;
        CHECK(at < 25);
        e->cells[at] = *cell;
        e->used[at] = true;
        e->enters++;
        e->live++;
    } else {
        int at = 0;
        while (at < 25 && (!e->used[at] || e->cells[at].x != cell->x || e->cells[at].z != cell->z))
            at++;
        CHECK(at < 25);
        e->used[at] = false;
        e->leaves++;
        e->live--;
    }
}
static void test_world(void) {
    RiptideWorldConfig c = riptide_world_defaults(17);
    c.water_threshold = 0;
    static const int dx[] = {0, 1, 0, -1}, dz[] = {1, 0, -1, 0};
    for (int z = -10; z <= 10; z++)
        for (int x = -10; x <= 10; x++) {
            RiptideCell a, b;
            CHECK(riptide_world_cell(&c, x, z, &a));
            CHECK(riptide_world_cell(&c, x, z, &b));
            CHECK(a.variant == b.variant && a.roads == b.roads && a.height == b.height);
            for (int d = 0; d < 4; d++) {
                CHECK(riptide_world_cell(&c, x + dx[d], z + dz[d], &b));
                CHECK(((a.roads >> d) & 1) == ((b.roads >> ((d + 2) % 4)) & 1));
            }
        }
    CHECK(!riptide_world_cell(&c, RIPTIDE_CELL_LIMIT + 1, 0, &(RiptideCell){0}));
    CHECK(riptide_hash_cell(2, -1, 0, 4) != riptide_hash_cell(2, INT64_C(4294967295), 0, 4));
    for (int x = -5; x <= 5; x++) {
        float a = riptide_world_height(&c, x * 64 - .0001, 17),
              b = riptide_world_height(&c, x * 64 + .0001, 17);
        CHECK(fabsf(a - b) < .001f);
    }
    RiptideOrigin o = {100, -200}, next = {103, -198};
    RiptideVec3 points[] = {{14, 7, 5}, {-8, 3, 9}};
    double global = points[0].x + (double)o.x * 64;
    CHECK(riptide_world_rebase(&o, next, 64, points, 2));
    CHECK(fabs(points[0].x + (double)o.x * 64 - global) < .001);
    CHECK(points[0].y == 7);
    Events e = {0};
    RiptideStream *s = riptide_stream_create(c, 2, event, &e);
    CHECK(s);
    CHECK(riptide_stream_update(s, -2, -3, 3) == 3);
    CHECK(e.live == 3);
    CHECK(riptide_stream_pending(s) == 22);
    while (riptide_stream_pending(s))
        riptide_stream_update(s, -2, -3, 3);
    CHECK(e.live == 25);
    CHECK(riptide_stream_update(s, -2, -3, 25) == 0);
    CHECK(riptide_stream_update(s, -1, -3, 25) == 5);
    CHECK(e.live == 25);
    CHECK(riptide_stream_update(s, 100, 100, 0) == 0);
    CHECK(e.live == 0);
    CHECK(riptide_stream_update(s, 100, 100, 25) == 25);
    riptide_stream_destroy(s);
    CHECK(e.live == 0);
    CHECK(e.enters == e.leaves);
}
static void test_route(void) {
    RiptideRoadNode nodes[] = {
        {{0, 0, 0}, 0, 2}, {{0, 0, 0}, 2, 1}, {{0, 0, 0}, 3, 1}, {{0, 0, 0}, 4, 0}};
    RiptideRoadEdge edges[] = {{1, 10}, {2, 1}, {3, 1}, {1, 1}};
    RiptideRoadGraph graph = {nodes, 4, edges, 4};
    RiptideRouteWork work[4];
    uint32_t path[4] = {99, 99, 99, 99};
    CHECK(riptide_route_find(&graph, 0, 3, work, 4, path, 2) == 4);
    CHECK(path[0] == 99);
    CHECK(riptide_route_find(&graph, 0, 3, work, 4, path, 4) == 4);
    CHECK(path[0] == 0 && path[1] == 2 && path[2] == 1 && path[3] == 3);
    CHECK(riptide_route_find(&graph, 3, 0, work, 4, path, 4) == 0);
    CHECK(riptide_route_find(&graph, 2, 2, work, 4, path, 4) == 1);
    CHECK(path[0] == 2);
    edges[0].cost = -1;
    CHECK(riptide_route_find(&graph, 0, 3, work, 4, path, 4) == 0);
    CHECK(riptide_route_steering((RiptideVec3){0}, 0, (RiptideVec3){1, 0, 0}, 1) > 0);
}
typedef struct {
    int index, result;
    unsigned *completed;
} JobData;
static void job_work(void *ptr) {
    JobData *j = ptr;
    j->result = j->index * j->index;
}
static void job_complete(void *ptr) {
    JobData *j = ptr;
    CHECK(j->result == j->index * j->index);
    (*j->completed)++;
}
static void test_jobs(void) {
    RiptideJobs *pool = riptide_jobs_create(4, 128);
    CHECK(pool);
    JobData jobs[128];
    unsigned completed = 0;
    for (int i = 0; i < 128; i++) {
        jobs[i] = (JobData){i, -1, &completed};
        CHECK(riptide_jobs_submit(pool, job_work, job_complete, &jobs[i]));
    }
    CHECK(completed == 0);
    CHECK(!riptide_jobs_submit(pool, job_work, NULL, &jobs[0]));
    unsigned released = riptide_jobs_poll(pool, 10);
    CHECK(released <= 10);
    CHECK(completed == released);
    riptide_jobs_destroy(pool);
    CHECK(completed == 128);
    CHECK(!riptide_jobs_create(0, 1));
    CHECK(!riptide_jobs_create(1, 0));
}
static void test_physics(void) {
    RiptideBody a = {.position = {0, 10, 0}, .inverse_mass = 0},
           b = {.position = {10, 10, 0}, .inverse_mass = 1};
    CHECK(near(riptide_distance_solve(&a, &b, 2, 1, false), 8, .001f));
    CHECK(near(b.position.x, 2, .001f));
    CHECK(a.position.x == 0);
    b.position.x = 1;
    CHECK(riptide_distance_solve(&a, &b, 2, 1, true) == 0);
    CHECK(b.position.x == 1);
    b.radius = .3f;
    for (int i = 0; i < 1200; i++)
        riptide_body_step(&b, (RiptideVec3){0, -18, 0}, .1f, .3f, NULL, NULL, 1.f / 120);
    CHECK(near(b.position.y, .3f, .001f));
    CHECK(isfinite(b.velocity.y));
}
static void test_render(void) {
    RiptideRenderer *r = riptide_render_create(10000);
    RiptideCanvas *canvas = riptide_canvas_create(64, 64);
    CHECK(r && canvas);
    RiptideCamera camera = {.eye = {0, 0, -5},
                       .target = {0, 0, 0},
                       .vertical_fov = 1,
                       .near_clip = .2f,
                       .far_clip = 100,
                       .bloom_layers = 2};
    CHECK(riptide_render_begin(r, 64, 64, camera));
    riptide_render_quad(r, (RiptideVec3){-1, -1, 0}, (RiptideVec3){1, -1, 0}, (RiptideVec3){1, 1, 0},
                   (RiptideVec3){-1, 1, 0}, (RiptideColor){1, 0, 0, 1});
    riptide_render_quad(r, (RiptideVec3){-1, -1, 1}, (RiptideVec3){1, -1, 1}, (RiptideVec3){1, 1, 1},
                   (RiptideVec3){-1, 1, 1}, (RiptideColor){0, 0, 1, 1});
    CHECK(riptide_canvas_draw(canvas, r, (RiptideColor){0}));
    const unsigned char *pixels = riptide_canvas_pixels(canvas);
    size_t centre = (32 * 64 + 32) * 4;
    CHECK(pixels[centre] > 250 && pixels[centre + 2] < 5);
    riptide_render_rect(r, 0, 0, 8, 8, (RiptideColor){0, 1, 0, 1});
    CHECK(riptide_canvas_draw(canvas, r, (RiptideColor){0}));
    CHECK(pixels[(2 * 64 + 2) * 4 + 1] > 250);
    CHECK(!riptide_render_begin(r, 0, 64, camera));
    CHECK(riptide_render_begin(r, 64, 64, camera));
    riptide_render_line(r, (RiptideVec3){-1, 0, -5}, (RiptideVec3){1, 0, 4}, (RiptideColor){0, 1, 1, 1}, 2, true);
    CHECK(riptide_render_mesh(r, RIPTIDE_GLOW).count > 0);
    CHECK(riptide_canvas_draw(canvas, r, (RiptideColor){0}));
    CHECK(riptide_render_begin(r, 64, 64, camera));
    riptide_render_quad(r, (RiptideVec3){-1, -1, -8}, (RiptideVec3){1, -1, -8}, (RiptideVec3){1, 1, -8},
                   (RiptideVec3){-1, 1, -8}, (RiptideColor){1, 0, 0, 1});
    CHECK(riptide_canvas_draw(canvas, r, (RiptideColor){0}));
    CHECK(pixels[centre] == 0);
    riptide_canvas_destroy(canvas);
    riptide_render_destroy(r);
    r = riptide_render_create(3);
    CHECK(r);
    CHECK(riptide_render_begin(r, 64, 64, camera));
    riptide_render_rect(r, 0, 0, 10, 10, (RiptideColor){1, 1, 1, 1});
    CHECK(riptide_render_mesh(r, RIPTIDE_UI).dropped == 3);
    riptide_render_destroy(r);
}
static void test_image(void) {
    unsigned char rgba[] = {255, 0, 0, 255, 0, 255, 0, 255, 0, 0, 255, 255, 255, 255, 0, 255};
    FILE *f = tmpfile();
    CHECK(f);
    CHECK(riptide_image_write(f, rgba, 2, 2, 8, false, false));
    rewind(f);
    unsigned char signature[8];
    CHECK(fread(signature, 1, 8, f) == 8);
    CHECK(memcmp(signature, "\211PNG\r\n\032\n", 8) == 0);
    CHECK(fclose(f) == 0);
    f = tmpfile();
    CHECK(f);
    CHECK(riptide_image_write(f, rgba, 2, 2, 8, true, true));
    CHECK(fseek(f, 54, SEEK_SET) == 0);
    CHECK(fread(signature, 1, 4, f) == 4);
    CHECK(signature[0] == 255 && signature[1] == 0 && signature[2] == 0);
    CHECK(fclose(f) == 0);
    CHECK(!riptide_image_write(NULL, rgba, 2, 2, 8, false, false));
    CHECK(!riptide_image_save(NULL, rgba, 2, 2, 8, false));
}
static void test_gpu_cache(void) {
    RiptideGpuStream *s = riptide_gpu_stream_new();
    CHECK(s);
    riptide_gpu_stream_reset(s);
    RiptideGpuPrimitive p = {0};
    p.meta[0] = 0;
    CHECK(!riptide_gpu_cache_begin(s, 7, 99));
    riptide_gpu_stream_emit(s, 0, &p);
    riptide_gpu_cache_end(s);
    CHECK(s->fixed_count == 1 && s->pass[0].count == 1 && !s->failed);
    CHECK(s->dirty_count > 0);
    s->dirty_count = 0; /* Backend acknowledges a completed upload. */
    riptide_gpu_stream_reset(s);
    CHECK(riptide_gpu_cache_begin(s, 7, 99));
    CHECK(s->hits == 1 && s->pass[0].count == 1);
    riptide_gpu_stream_scene(s, 2);
    CHECK(s->fixed_count == 0 && s->pass[0].count == 0);
    riptide_gpu_stream_emit(s, 0, &p);
    CHECK(s->pass[0].v[0] & GPU_DYNAMIC_BIT);
    CHECK(riptide_gpu_stream_bytes(s) > sizeof *s);
    riptide_gpu_stream_free(s);
    s = riptide_gpu_stream_new();
    CHECK(s);
    riptide_gpu_stream_emit(s, 9, &p);
    CHECK(s->failed);
    riptide_gpu_stream_free(s);
}
int main(void) {
    test_clock();
    test_vehicle();
    test_world();
    test_route();
    test_jobs();
    test_physics();
    test_render();
    test_image();
    test_gpu_cache();
    printf("Riptide Engine: %u checks passed\n", checks);
    return 0;
}
