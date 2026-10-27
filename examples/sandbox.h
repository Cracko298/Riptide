/* SPDX-License-Identifier: MIT */
#ifndef RIPTIDE_EXAMPLE_SANDBOX_H
#define RIPTIDE_EXAMPLE_SANDBOX_H
#include "riptide/riptide.h"
#define SANDBOX_TRAFFIC 16
#define SANDBOX_CELLS 81
typedef struct {
    RiptideWorldConfig world;
    RiptideOrigin origin;
    RiptideStream *stream;
    RiptideCell cells[SANDBOX_CELLS];
    bool resident[SANDBOX_CELLS];
    RiptideVehicleConfig handling;
    RiptideVehicle player, traffic[SANDBOX_TRAFFIC];
    RiptidePose previous;
    RiptideBody chain[5];
    RiptideRenderer *renderer;
    RiptideCanvas *canvas;
    unsigned width, height;
    float time;
    unsigned collisions;
} Sandbox;
/* Example-owned lifecycle; these are not part of the engine API. */
bool sandbox_init(Sandbox *sandbox, unsigned width, unsigned height);
void sandbox_step(Sandbox *sandbox, RiptideControls input, float dt);
bool sandbox_draw(Sandbox *sandbox, float alpha);
void sandbox_free(Sandbox *sandbox);
#endif
