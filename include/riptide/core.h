/* SPDX-License-Identifier: MIT */
#ifndef RIPTIDE_CORIPTIDE_H
#define RIPTIDE_CORIPTIDE_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
#define RIPTIDE_PI 3.14159265358979323846f
#define RIPTIDE_VERSION "0.1.0"
typedef struct {
    float x, y, z;
} RiptideVec3;
typedef struct {
    float r, g, b, a;
} RiptideColor;
typedef struct {
    RiptideVec3 position;
    float yaw, pitch, roll;
} RiptidePose;
typedef struct {
    double step, accumulator, dropped_seconds;
    unsigned max_steps;
} RiptideClock;
/** Clamp a finite value to an inclusive interval.
 * @param value Input value.
 * @param lo Lower bound, no greater than hi.
 * @param hi Upper bound.
 * @return The clamped value. */
float riptide_clamp(float value, float lo, float hi);
/** Find the signed shortest rotation from b to a.
 * @param a Target angle in radians.
 * @param b Reference angle in radians.
 * @return Angle in [-pi, pi). */
float riptide_angle_delta(float a, float b);
/** Hash a global cell without mutable random state.
 * @param seed World seed.
 * @param x Signed global cell X coordinate.
 * @param z Signed global cell Z coordinate.
 * @param salt Separates independent procedural features.
 * @return A reproducible 32-bit value; not a cryptographic hash. */
uint32_t riptide_hash_cell(uint32_t seed, int64_t x, int64_t z, uint32_t salt);
/** Initialize a fixed-step clock.
 * @param clock Output clock; reset on success.
 * @param hz Simulation frequency in [1, 1000] Hz.
 * @param max_steps Maximum catch-up steps per frame, in [1, 1000].
 * @return False for invalid arguments. */
bool riptide_clock_init(RiptideClock *clock, double hz, unsigned max_steps);
/** Accumulate real time and consume the available simulation steps.
 * @param clock Initialized clock.
 * @param elapsed Nonnegative elapsed seconds; nonfinite values are ignored.
 * @return Number of steps to run with clock->step seconds each.
 * Excess backlog is discarded and counted in dropped_seconds. */
unsigned riptide_clock_advance(RiptideClock *clock, double elapsed);
/** Get the render interpolation fraction after riptide_clock_advance.
 * @param clock Initialized clock.
 * @return A fraction in [0, 1), or zero for an invalid clock. */
float riptide_clock_alpha(const RiptideClock *clock);
/** Interpolate presentation without modifying simulation state.
 * @param previous Pose before the latest fixed step.
 * @param current Pose after that step; do not interpolate across teleports.
 * @param alpha Blend fraction, clamped to [0, 1].
 * @return Interpolated position and shortest-path angles. */
RiptidePose riptide_pose_lerp(RiptidePose previous, RiptidePose current, float alpha);
#ifdef __cplusplus
}
#endif
#endif
