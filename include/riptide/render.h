/* SPDX-License-Identifier: MIT */
#ifndef RIPTIDE_RENDER_H
#define RIPTIDE_RENDER_H
#include "core.h"
#ifdef __cplusplus
extern "C" {
#endif
typedef enum { RIPTIDE_SOLID, RIPTIDE_EDGE, RIPTIDE_GLOW, RIPTIDE_UI, RIPTIDE_PASS_COUNT } RiptidePass;
/** Homogeneous clip vertex: -w<=x,y<=w and -w<=z<=0, with reversed depth. */
typedef struct {
    float x, y, z, w;
    RiptideColor c;
} RiptideVertex;
typedef struct {
    const RiptideVertex *vertices;
    size_t count, dropped;
} RiptideMeshView;
typedef struct RiptideRenderer RiptideRenderer;
typedef struct RiptideCanvas RiptideCanvas;
typedef struct {
    RiptideVec3 eye, target;
    float vertical_fov, near_clip, far_clip; /**< Radians and metres. */
    RiptideColor fog;
    bool fog_enabled;
    unsigned bloom_layers; /**< Outline halo layers, [0, 3]. */
} RiptideCamera;
/** Allocate fixed-capacity triangle buffers.
 * @param vertices_per_pass Maximum vertices in each pass, [3, 4000000].
 * @return Owned renderer or NULL. Capacity is fixed; excess vertices are counted, not allocated. */
RiptideRenderer *riptide_render_create(size_t vertices_per_pass);
/** Begin a frame and set the perspective camera.
 * @param renderer Renderer to reset.
 * @param width Viewport width in pixels, [1, 8192].
 * @param height Viewport height in pixels, [1, 8192].
 * @param camera Camera copied by value; target must differ from eye.
 * @return False for invalid dimensions/camera, leaving the prior frame intact. */
bool riptide_render_begin(RiptideRenderer *renderer, unsigned width, unsigned height, RiptideCamera camera);
/** Append a world-space quad as two solid triangles.
 * @param renderer Renderer after a successful begin.
 * @param a First corner in metres.
 * @param b Second corner in winding order.
 * @param c Third corner in winding order.
 * @param d Fourth corner in winding order; repeat c to emit one degenerate triangle.
 * @param color RGBA components in [0, 1]. Transparency is submission-ordered.
 * Terrain heights are explicit; this function never samples the world. */
void riptide_render_quad(RiptideRenderer *renderer, RiptideVec3 a, RiptideVec3 b, RiptideVec3 c, RiptideVec3 d, RiptideColor color);
/** Append a camera-facing ribbon with optional additive halo layers.
 * @param renderer Active renderer.
 * @param a World-space start.
 * @param b World-space end.
 * @param color Line RGBA.
 * @param width Ribbon width in pixels, greater than zero.
 * @param glow Whether to emit halos using camera.bloom_layers.
 * Lines are clipped at the near plane before expansion. */
void riptide_render_line(RiptideRenderer *renderer, RiptideVec3 a, RiptideVec3 b, RiptideColor color, float width,
                    bool glow);
/** Append a shaded, optionally outlined box.
 * @param renderer Active renderer.
 * @param pose Centre and rotation; all three angles are applied.
 * @param size Full X/Y/Z dimensions in metres.
 * @param fill Face RGBA; zero alpha suppresses faces.
 * @param edge Edge RGBA; zero alpha suppresses edges.
 * @param glow Enable outline halos. */
void riptide_render_box(RiptideRenderer *renderer, RiptidePose pose, RiptideVec3 size, RiptideColor fill, RiptideColor edge,
                   bool glow);
/** Append a screen-space UI rectangle.
 * @param renderer Active renderer.
 * @param x Left edge in pixels.
 * @param y Top edge in pixels.
 * @param width Rectangle width in pixels.
 * @param height Rectangle height in pixels.
 * @param color Rectangle RGBA, blended after world passes. */
void riptide_render_rect(RiptideRenderer *renderer, float x, float y, float width, float height,
                    RiptideColor color);
/** Borrow the triangles of one pass for a graphics backend.
 * @param renderer Active renderer.
 * @param pass Requested pass.
 * @return Read-only view, valid until the next mutation or destruction.
 * Draw passes in enum order. Only RIPTIDE_SOLID writes depth; RIPTIDE_GLOW blends additively. */
RiptideMeshView riptide_render_mesh(const RiptideRenderer *renderer, RiptidePass pass);
/** Release renderer storage.
 * @param renderer Owned renderer; NULL is accepted. */
void riptide_render_destroy(RiptideRenderer *renderer);
/** Allocate a reusable CPU framebuffer and depth storage.
 * @param width Width in [1, 8192] pixels.
 * @param height Height in [1, 8192] pixels; total pixels must not exceed 32 million.
 * @return Owned canvas or NULL. No per-frame allocation is required. */
RiptideCanvas *riptide_canvas_create(unsigned width, unsigned height);
/** Rasterize all submitted triangles with clipping and depth testing.
 * @param canvas Destination canvas.
 * @param renderer Source renderer; viewport must match the canvas.
 * @param clear Background RGBA; output alpha is always opaque.
 * @return False for NULL arguments or mismatched dimensions.
 * This reference backend favours portability over GPU-level performance. */
bool riptide_canvas_draw(RiptideCanvas *canvas, const RiptideRenderer *renderer, RiptideColor clear);
/** Borrow the rendered RGBA8 framebuffer, top row first.
 * @param canvas Rendered canvas.
 * @return Pixels with tightly packed width*4 byte rows, or NULL.
 * Storage remains owned by canvas and changes on the next draw. */
const unsigned char *riptide_canvas_pixels(const RiptideCanvas *canvas);
/** Release canvas buffers.
 * @param canvas Owned canvas; NULL is accepted. */
void riptide_canvas_destroy(RiptideCanvas *canvas);
#ifdef __cplusplus
}
#endif
#endif
