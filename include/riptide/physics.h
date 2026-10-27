/* SPDX-License-Identifier: MIT */
#ifndef RIPTIDE_PHYSICS_H
#define RIPTIDE_PHYSICS_H
#include "vehicle.h"
#ifdef __cplusplus
extern "C" {
#endif
typedef struct {
    RiptideVec3 position, velocity;
    float inverse_mass, radius;
} RiptideBody;
/** Integrate a simple debris/attachment body against a heightfield.
 * @param body Mutable body; zero inverse_mass makes it static.
 * @param gravity World acceleration in m/s^2.
 * @param damping Linear velocity damping in 1/s, nonnegative.
 * @param restitution Ground bounce in [0, 1].
 * @param height Ground callback, or NULL for y=0.
 * @param user Borrowed callback context.
 * @param dt Fixed timestep in (0, 0.05] seconds.
 * Radius sets ground clearance; rotational dynamics are not modelled. */
void riptide_body_step(RiptideBody *body, RiptideVec3 gravity, float damping, float restitution, RiptideHeightFn height,
                  void *user, float dt);
/** Project a distance constraint and remove relative radial velocity.
 * @param a First body; distinct from b.
 * @param b Second body.
 * @param length Rest length in metres, nonnegative.
 * @param stiffness Per-iteration correction fraction in [0, 1].
 * @param rope True allows slack; false constrains both compression and extension.
 * @return Absolute positional error before correction, in metres.
 * Integrate all bodies first, then solve constraints for several iterations.
 * Anchors are body centres; this is a positional joint, not a rigid-body hinge. */
float riptide_distance_solve(RiptideBody *a, RiptideBody *b, float length, float stiffness, bool rope);
#ifdef __cplusplus
}
#endif
#endif
