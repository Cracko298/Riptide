/* SPDX-License-Identifier: MIT */
#include "render_internal.h"
#include <math.h>
#include <stdlib.h>
static RiptideVec3 sub(RiptideVec3 a, RiptideVec3 b) {
    return (RiptideVec3){a.x - b.x, a.y - b.y, a.z - b.z};
}
static float dot(RiptideVec3 a, RiptideVec3 b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}
static RiptideVec3 cross(RiptideVec3 a, RiptideVec3 b) {
    return (RiptideVec3){a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
static RiptideVec3 norm(RiptideVec3 a) {
    float l = sqrtf(dot(a, a));
    return (RiptideVec3){a.x / l, a.y / l, a.z / l};
}
static bool finite_vec(RiptideVec3 v) {
    return isfinite(v.x) && isfinite(v.y) && isfinite(v.z);
}
static RiptideColor fogged(const RiptideRenderer *r, RiptideColor c, float depth) {
    if (!r->camera.fog_enabled)
        return c;
    float t = riptide_clamp((depth - r->camera.far_clip * .35f) / (r->camera.far_clip * .65f), 0, .98f);
    c.r += (r->camera.fog.r - c.r) * t;
    c.g += (r->camera.fog.g - c.g) * t;
    c.b += (r->camera.fog.b - c.b) * t;
    return c;
}
RiptideRenderer *riptide_render_create(size_t cap) {
    if (cap < 3 || cap > 4000000)
        return NULL;
    RiptideRenderer *r = calloc(1, sizeof *r);
    if (!r)
        return NULL;
    for (int i = 0; i < RIPTIDE_PASS_COUNT; i++) {
        r->mesh[i].v = malloc(cap * sizeof(RiptideVertex));
        r->mesh[i].capacity = cap;
        if (!r->mesh[i].v) {
            riptide_render_destroy(r);
            return NULL;
        }
    }
    return r;
}
void riptide_render_destroy(RiptideRenderer *r) {
    if (!r)
        return;
    for (int i = 0; i < RIPTIDE_PASS_COUNT; i++)
        free(r->mesh[i].v);
    free(r);
}
bool riptide_render_begin(RiptideRenderer *r, unsigned w, unsigned h, RiptideCamera c) {
    if (!r || !w || !h || w > 8192 || h > 8192 || !finite_vec(c.eye) || !finite_vec(c.target) ||
        !isfinite(c.vertical_fov) || c.vertical_fov <= .05f || c.vertical_fov >= 3.1f ||
        !isfinite(c.near_clip) || !isfinite(c.far_clip) || c.near_clip <= 0 ||
        c.far_clip <= c.near_clip || c.bloom_layers > 3)
        return false;
    RiptideVec3 forward = sub(c.target, c.eye);
    if (dot(forward, forward) < 1e-8f)
        return false;
    forward = norm(forward);
    RiptideVec3 right = cross((RiptideVec3){0, 1, 0}, forward);
    if (dot(right, right) < 1e-6f)
        right = cross((RiptideVec3){0, 0, 1}, forward);
    r->right = norm(right);
    r->up = cross(forward, r->right);
    r->forward = forward;
    r->camera = c;
    r->width = w;
    r->height = h;
    r->focal = 1 / tanf(c.vertical_fov * .5f);
    for (int i = 0; i < RIPTIDE_PASS_COUNT; i++)
        r->mesh[i].count = r->mesh[i].dropped = 0;
    return true;
}
static void tri(RiptideRenderer *r, RiptidePass pass, RiptideVertex a, RiptideVertex b, RiptideVertex c) {
    RiptideMesh *m = &r->mesh[pass];
    if (m->count + 3 > m->capacity) {
        m->dropped += 3;
        return;
    }
    m->v[m->count++] = a;
    m->v[m->count++] = b;
    m->v[m->count++] = c;
}
static RiptideVertex project(RiptideRenderer *r, RiptideVec3 p, RiptideColor c) {
    RiptideVec3 d = sub(p, r->camera.eye);
    float z = dot(d, r->forward);
    float near = r->camera.near_clip, far = r->camera.far_clip;
    return (RiptideVertex){dot(d, r->right) * r->focal * r->height / r->width, dot(d, r->up) * r->focal,
                      near * (z - far) / (far - near), z, fogged(r, c, z)};
}
void riptide_render_quad(RiptideRenderer *r, RiptideVec3 a, RiptideVec3 b, RiptideVec3 c, RiptideVec3 d, RiptideColor color) {
    if (!r || !r->width)
        return;
    RiptideVertex aa = project(r, a, color), bb = project(r, b, color), cc = project(r, c, color),
             dd = project(r, d, color);
    tri(r, RIPTIDE_SOLID, aa, bb, cc);
    tri(r, RIPTIDE_SOLID, aa, cc, dd);
}
static void strip(RiptideRenderer *r, RiptideVertex a, RiptideVertex b, float width, RiptidePass pass, RiptideColor color,
                  float nx, float ny) {
    float ox = nx * width / r->width, oy = ny * width / r->height;
    RiptideVertex c = a, d = b;
    a.x += ox * a.w;
    a.y += oy * a.w;
    b.x += ox * b.w;
    b.y += oy * b.w;
    c.x -= ox * c.w;
    c.y -= oy * c.w;
    d.x -= ox * d.w;
    d.y -= oy * d.w;
    a.c = b.c = c.c = d.c = color;
    tri(r, pass, a, b, d);
    tri(r, pass, a, d, c);
}
void riptide_render_line(RiptideRenderer *r, RiptideVec3 from, RiptideVec3 to, RiptideColor color, float width, bool glow) {
    if (!r || !r->width || !isfinite(width) || width <= 0)
        return;
    RiptideVertex a = project(r, from, color), b = project(r, to, color);
    float near = r->camera.near_clip;
    if (a.w < near && b.w < near)
        return;
    if (a.w < near || b.w < near) {
        float t = (near - a.w) / (b.w - a.w);
        RiptideVertex v = a;
        v.x += (b.x - a.x) * t;
        v.y += (b.y - a.y) * t;
        v.z += (b.z - a.z) * t;
        v.w = near;
        if (a.w < near)
            a = v;
        else
            b = v;
    }
    color = fogged(r, color, (a.w + b.w) * .5f);
    a.z = fmaxf(-a.w, a.z - .00020f);
    b.z = fmaxf(-b.w, b.z - .00020f);
    float dx = (b.x / b.w - a.x / a.w) * r->width, dy = (b.y / b.w - a.y / a.w) * r->height;
    float len = hypotf(dx, dy);
    if (len < .001f)
        return;
    float nx = -dy / len, ny = dx / len;
    strip(r, a, b, width, RIPTIDE_EDGE, color, nx, ny);
    if (glow)
        for (unsigned i = r->camera.bloom_layers; i > 0; i--) {
            RiptideColor halo = color;
            halo.a *= i == 1 ? .20f : i == 2 ? .075f : .035f;
            strip(r, a, b, width + 3.8f * i, RIPTIDE_GLOW, halo, nx, ny);
        }
}
static RiptideVec3 transform(RiptideVec3 p, RiptidePose pose) {
    float s = sinf(pose.roll), c = cosf(pose.roll), x = p.x * c - p.y * s, y = p.x * s + p.y * c;
    p.x = x;
    p.y = y;
    s = sinf(pose.pitch);
    c = cosf(pose.pitch);
    y = p.y * c - p.z * s;
    float z = p.y * s + p.z * c;
    p.y = y;
    p.z = z;
    s = sinf(pose.yaw);
    c = cosf(pose.yaw);
    return (RiptideVec3){pose.position.x + p.x * c + p.z * s, pose.position.y + p.y,
                    pose.position.z + p.z * c - p.x * s};
}
void riptide_render_box(RiptideRenderer *r, RiptidePose pose, RiptideVec3 size, RiptideColor fill, RiptideColor edge, bool glow) {
    if (!r || !r->width)
        return;
    RiptideVec3 p[8];
    for (int i = 0; i < 8; i++)
        p[i] = transform((RiptideVec3){(i & 1 ? .5f : -.5f) * size.x, (i & 2 ? .5f : -.5f) * size.y,
                                  (i & 4 ? .5f : -.5f) * size.z},
                         pose);
    static const int faces[6][4] = {{0, 1, 3, 2}, {4, 6, 7, 5}, {0, 2, 6, 4},
                                    {1, 5, 7, 3}, {2, 3, 7, 6}, {0, 4, 5, 1}};
    static const float shades[] = {.6f, .8f, .55f, .75f, 1, .4f};
    if (fill.a > 0)
        for (int i = 0; i < 6; i++) {
            RiptideColor c = fill;
            c.r *= shades[i];
            c.g *= shades[i];
            c.b *= shades[i];
            riptide_render_quad(r, p[faces[i][0]], p[faces[i][1]], p[faces[i][2]], p[faces[i][3]], c);
        }
    static const int edges[12][2] = {{0, 1}, {2, 3}, {4, 5}, {6, 7}, {0, 2}, {1, 3},
                                     {4, 6}, {5, 7}, {0, 4}, {1, 5}, {2, 6}, {3, 7}};
    if (edge.a > 0)
        for (int i = 0; i < 12; i++)
            riptide_render_line(r, p[edges[i][0]], p[edges[i][1]], edge, 1, glow);
}
void riptide_render_rect(RiptideRenderer *r, float x, float y, float w, float h, RiptideColor color) {
    if (!r || !r->width)
        return;
    RiptideVertex a = {2 * x / r->width - 1, 1 - 2 * y / r->height, -.5f, 1, color}, b = a, c = a, d = a;
    b.x += 2 * w / r->width;
    c.x = b.x;
    c.y -= 2 * h / r->height;
    d.y = c.y;
    tri(r, RIPTIDE_UI, a, b, c);
    tri(r, RIPTIDE_UI, a, c, d);
}
RiptideMeshView riptide_render_mesh(const RiptideRenderer *r, RiptidePass pass) {
    if (!r || pass < 0 || pass >= RIPTIDE_PASS_COUNT)
        return (RiptideMeshView){0};
    const RiptideMesh *m = &r->mesh[pass];
    return (RiptideMeshView){m->v, m->count, m->dropped};
}
