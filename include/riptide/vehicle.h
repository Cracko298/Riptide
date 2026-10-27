/* SPDX-License-Identifier: MIT */
#ifndef RIPTIDE_VEHICLE_H
#define RIPTIDE_VEHICLE_H
#include "core.h"
#ifdef __cplusplus
extern "C" {
#endif
/** Arcade handling parameters. Distances are metres; times are seconds. */
typedef struct {
    float length, width, height, mass;       /**< Dimensions and mass in kg. */
    float max_speed, reverse_speed;          /**< m/s. */
    float acceleration, braking;             /**< m/s^2. */
    float grip, drift_grip, drag;            /**< Lateral damping /s, drift multiplier, drag /s. */
    float steering_angle, steering_response; /**< Maximum radians, response /s. */
    float yaw_limit, drift_min_speed;        /**< rad/s and m/s. */
    float boost_acceleration, boost_speed;   /**< Additional m/s^2 and m/s. */
    float wheel_radius, suspension_softness;
} RiptideVehicleConfig;
typedef struct {
    float throttle; /**< [-1, 1]; negative requests reverse. */
    float brake;    /**< [0, 1]; brake never requests reverse. */
    float steering; /**< [-1, 1]; positive turns toward +X from +Z. */
    bool drift, boost;
} RiptideControls;
typedef struct {
    RiptidePose pose;
    RiptideVec3 velocity;
    float steering, wheel_spin, tiriptide_slip, brake_light;
    float heave, heave_rate, pitch_rate, roll_rate;
    bool grounded;
} RiptideVehicle;
/** Return a generic hatchback configuration with no game progression.
 * @return A valid starting configuration; callers may change individual fields. */
RiptideVehicleConfig riptide_vehicle_defaults(void);
/** Check a handling configuration before installing it.
 * @param config Parameters to validate.
 * @return True when all fields are finite and within the documented API bounds.
 * See docs/API.md for bounds. */
bool riptide_vehicle_config_valid(const RiptideVehicleConfig *config);
/** Place a vehicle and clear its motion and suspension.
 * @param vehicle Output state.
 * @param position World-space ground contact position in metres.
 * @param yaw Heading in radians; zero faces +Z.
 * A null vehicle is ignored. */
void riptide_vehicle_reset(RiptideVehicle *vehicle, RiptideVec3 position, float yaw);
/** Sample the driving surface.
 * @param user Application-owned context, passed through unchanged.
 * @param x World X in metres.
 * @param z World Z in metres.
 * @return Ground height in metres; must be finite. */
typedef float (*RiptideHeightFn)(void *user, float x, float z);
/** Advance one arcade driving step, including gravity and visual suspension.
 * @param vehicle Mutable vehicle state.
 * @param config Valid configuration; no pointer is retained.
 * @param controls Input values, clamped to their ranges.
 * @param height Ground callback; NULL selects a flat surface at y=0.
 * @param user Context borrowed only during height calls.
 * @param dt Fixed timestep in (0, 0.05] seconds.
 * @return False for invalid parameters, leaving vehicle unchanged.
 * Collision resolution is separate; use small fixed steps to limit tunnelling. */
bool riptide_vehicle_step(RiptideVehicle *vehicle, const RiptideVehicleConfig *config, RiptideControls controls,
                     RiptideHeightFn height, void *user, float dt);
/** Apply a velocity impulse at the centre of mass.
 * @param vehicle Mutable state.
 * @param config Valid configuration, including mass in kg.
 * @param impulse World-space impulse in kg*m/s. Positive Y launches the vehicle.
 * No angular impulse is modelled. Null arguments are ignored. */
void riptide_vehicle_impulse(RiptideVehicle *vehicle, const RiptideVehicleConfig *config, RiptideVec3 impulse);
/** Measure horizontal travel speed.
 * @param vehicle Vehicle state; may be NULL.
 * @return m/s, or zero for NULL. */
float riptide_vehicle_speed(const RiptideVehicle *vehicle);
/** Resolve two overlapping disc bodies in XZ with an impulse and separation.
 * @param a First vehicle, distinct from b.
 * @param ca Valid handling configuration for a.
 * @param b Second vehicle.
 * @param cb Valid handling configuration for b.
 * @param restitution Bounce coefficient, clamped to [0, 1].
 * @return Closing speed in m/s before the impulse; zero if separating or not touching.
 * Radius is half the vehicle width. Use multiple discs for long vehicles. */
float riptide_vehicle_collide(RiptideVehicle *a, const RiptideVehicleConfig *ca, RiptideVehicle *b,
                         const RiptideVehicleConfig *cb, float restitution);
/** Resolve one vehicle disc against an axis-aligned box in XZ.
 * @param vehicle Mutable state; ground height is unchanged.
 * @param config Valid configuration; radius is half width.
 * @param minimum Box minimum X/Z; Y is ignored.
 * @param maximum Box maximum X/Z; Y is ignored.
 * @param restitution Bounce coefficient in [0, 1].
 * @return Incoming normal speed in m/s, or zero if no impact.
 * This is discrete contact, not a swept collision test. */
float riptide_vehicle_collide_box(RiptideVehicle *vehicle, const RiptideVehicleConfig *config, RiptideVec3 minimum,
                             RiptideVec3 maximum, float restitution);
#ifdef __cplusplus
}
#endif
#endif
