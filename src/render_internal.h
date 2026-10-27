/* SPDX-License-Identifier: MIT */
#ifndef RIPTIDE_RENDER_INTERNAL_H
#define RIPTIDE_RENDER_INTERNAL_H
#include "riptide/render.h"
typedef struct {
    RiptideVertex *v;
    size_t count, capacity, dropped;
} RiptideMesh;
struct RiptideRenderer {
    RiptideMesh mesh[RIPTIDE_PASS_COUNT];
    RiptideCamera camera;
    RiptideVec3 right, up, forward;
    float focal;
    unsigned width, height;
};
struct RiptideCanvas {
    unsigned width, height;
    float *rgb, *depth;
    unsigned char *rgba;
};
#endif
