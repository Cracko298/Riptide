/* SPDX-License-Identifier: MIT */
#include "sandbox.h"
#include <stdio.h>
#include <math.h>
int main(int argc, char **argv) {
    const char *path = argc > 1 ? argv[1] : "sandbox.png";
    Sandbox s;
    if (!sandbox_init(&s, 800, 500)) {
        fputs("Initialization failed\n", stderr);
        return 1;
    }
    for (int i = 0; i < 240; i++)
        sandbox_step(&s, (RiptideControls){.throttle = .8f}, 1.f / 120);
    bool ok = sandbox_draw(&s, 1);
    for (int p = 0; p < RIPTIDE_PASS_COUNT; p++) {
        RiptideMeshView mesh = riptide_render_mesh(s.renderer, (RiptidePass)p);
        printf("pass %d: %zu vertices, %zu dropped\n", p, mesh.count, mesh.dropped);
        if (mesh.dropped)
            ok = false;
    }
    ok = ok &&
         riptide_image_save(path, riptide_canvas_pixels(s.canvas), s.width, s.height, s.width * 4, false);
    printf("%s: %s; speed %.1f km/h\n", path, ok ? "written" : "failed",
           riptide_vehicle_speed(&s.player) * 3.6f);
    sandbox_free(&s);
    return ok ? 0 : 1;
}
