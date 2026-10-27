/* SPDX-License-Identifier: MIT */
#include "render_internal.h"
#include <stdlib.h>
#include <math.h>
#include <string.h>
static float plane(RiptideVertex v, int p) {
    switch (p) {
    case 0:
        return v.x + v.w;
    case 1:
        return v.w - v.x;
    case 2:
        return v.y + v.w;
    case 3:
        return v.w - v.y;
    case 4:
        return v.z + v.w;
    default:
        return -v.z;
    }
}
static RiptideVertex lerp(RiptideVertex a, RiptideVertex b, float t) {
    return (RiptideVertex){a.x + (b.x - a.x) * t,
                      a.y + (b.y - a.y) * t,
                      a.z + (b.z - a.z) * t,
                      a.w + (b.w - a.w) * t,
                      {a.c.r + (b.c.r - a.c.r) * t, a.c.g + (b.c.g - a.c.g) * t,
                       a.c.b + (b.c.b - a.c.b) * t, a.c.a + (b.c.a - a.c.a) * t}};
}
static float edge(float ax, float ay, float bx, float by, float x, float y) {
    return (x - ax) * (by - ay) - (y - ay) * (bx - ax);
}
static void raster(RiptideVertex a, RiptideVertex b, RiptideVertex c, int pass, int w, int h, float *pixels,
                   float *depth) {
    if (a.w <= 0 || b.w <= 0 || c.w <= 0)
        return;
    float ax = (a.x / a.w + 1) * w * .5f, ay = (1 - a.y / a.w) * h * .5f,
          bx = (b.x / b.w + 1) * w * .5f, by = (1 - b.y / b.w) * h * .5f,
          cx = (c.x / c.w + 1) * w * .5f, cy = (1 - c.y / c.w) * h * .5f;
    float area = edge(ax, ay, bx, by, cx, cy);
    if (fabsf(area) < .00001f)
        return;
    int x0 = (int)riptide_clamp(floorf(fminf(ax, fminf(bx, cx))), 0, w - 1),
        x1 = (int)riptide_clamp(ceilf(fmaxf(ax, fmaxf(bx, cx))), 0, w - 1),
        y0 = (int)riptide_clamp(floorf(fminf(ay, fminf(by, cy))), 0, h - 1),
        y1 = (int)riptide_clamp(ceilf(fmaxf(ay, fmaxf(by, cy))), 0, h - 1);
    for (int y = y0; y <= y1; y++)
        for (int x = x0; x <= x1; x++) {
            float u = edge(bx, by, cx, cy, x + .5f, y + .5f) / area,
                  v = edge(cx, cy, ax, ay, x + .5f, y + .5f) / area, t = 1 - u - v;
            if (u < 0 || v < 0 || t < 0)
                continue;
            float z = -(u * a.z / a.w + v * b.z / b.w + t * c.z / c.w);
            int i = y * w + x;
            if (pass != RIPTIDE_UI && z + 1e-6f < depth[i])
                continue;
            float inv = u / a.w + v / b.w + t / c.w;
            RiptideColor col = {(u * a.c.r / a.w + v * b.c.r / b.w + t * c.c.r / c.w) / inv,
                           (u * a.c.g / a.w + v * b.c.g / b.w + t * c.c.g / c.w) / inv,
                           (u * a.c.b / a.w + v * b.c.b / b.w + t * c.c.b / c.w) / inv,
                           (u * a.c.a / a.w + v * b.c.a / b.w + t * c.c.a / c.w) / inv};
            float rgb[] = {col.r, col.g, col.b};
            for (int k = 0; k < 3; k++)
                pixels[i * 3 + k] = pass == RIPTIDE_GLOW
                                        ? pixels[i * 3 + k] + rgb[k] * col.a
                                        : pixels[i * 3 + k] * (1 - col.a) + rgb[k] * col.a;
            if (pass == RIPTIDE_SOLID)
                depth[i] = z;
        }
}
RiptideCanvas *riptide_canvas_create(unsigned w, unsigned h) {
    if (!w || !h || w > 8192 || h > 8192 || (size_t)w * h > 32000000)
        return NULL;
    RiptideCanvas *c = calloc(1, sizeof *c);
    if (!c)
        return NULL;
    size_t n = (size_t)w * h;
    c->width = w;
    c->height = h;
    c->rgb = malloc(n * 3 * sizeof(float));
    c->depth = malloc(n * sizeof(float));
    c->rgba = malloc(n * 4);
    if (!c->rgb || !c->depth || !c->rgba) {
        riptide_canvas_destroy(c);
        return NULL;
    }
    return c;
}
void riptide_canvas_destroy(RiptideCanvas *c) {
    if (c) {
        free(c->rgb);
        free(c->depth);
        free(c->rgba);
        free(c);
    }
}
const unsigned char *riptide_canvas_pixels(const RiptideCanvas *c) {
    return c ? c->rgba : NULL;
}
bool riptide_canvas_draw(RiptideCanvas *c, const RiptideRenderer *r, RiptideColor clear) {
    if (!c || !r || r->width != c->width || r->height != c->height)
        return false;
    size_t count = (size_t)c->width * c->height;
    memset(c->depth, 0, count * sizeof(float));
    for (size_t i = 0; i < count; i++) {
        c->rgb[i * 3] = clear.r;
        c->rgb[i * 3 + 1] = clear.g;
        c->rgb[i * 3 + 2] = clear.b;
    }
    for (int pass = 0; pass < RIPTIDE_PASS_COUNT; pass++) {
        const RiptideMesh *m = &r->mesh[pass];
        for (size_t k = 0; k + 2 < m->count; k += 3) {
            RiptideVertex poly[16], tmp[16];
            int n = 3;
            poly[0] = m->v[k];
            poly[1] = m->v[k + 1];
            poly[2] = m->v[k + 2];
            for (int p = 0; p < 6 && n; p++) {
                int nn = 0;
                for (int j = 0; j < n; j++) {
                    RiptideVertex a = poly[j], b = poly[(j + 1) % n];
                    float da = plane(a, p), db = plane(b, p);
                    if (da >= 0)
                        tmp[nn++] = a;
                    if ((da >= 0) != (db >= 0))
                        tmp[nn++] = lerp(a, b, da / (da - db));
                }
                n = nn;
                memcpy(poly, tmp, (size_t)n * sizeof *poly);
            }
            for (int j = 1; j < n - 1; j++)
                raster(poly[0], poly[j], poly[j + 1], pass, (int)c->width, (int)c->height, c->rgb,
                       c->depth);
        }
    }
    for (size_t i = 0; i < count; i++) {
        for (int k = 0; k < 3; k++)
            c->rgba[i * 4 + k] = (unsigned char)(riptide_clamp(c->rgb[i * 3 + k], 0, 1) * 255 + .5f);
        c->rgba[i * 4 + 3] = 255;
    }
    return true;
}
