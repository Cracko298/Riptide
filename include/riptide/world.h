/* SPDX-License-Identifier: MIT */
#ifndef RIPTIDE_WORLD_H
#define RIPTIDE_WORLD_H
#include "core.h"
#ifdef __cplusplus
extern "C" {
#endif
/** Supported global cell coordinates are [-2^40, 2^40]. */
#define RIPTIDE_CELL_LIMIT INT64_C(1099511627776)
typedef struct {
    uint32_t seed;
    float cell_size;         /**< Metres per cell, [8, 4096]. */
    float relief;            /**< Maximum elevation above water, [0, 1000] metres. */
    float water_level;       /**< Elevation datum in metres. */
    float water_threshold;   /**< Continental noise cutoff [0, 1]; not an exact area ratio. */
    unsigned artery_spacing; /**< Grid spacing [2, 64] cells. */
} RiptideWorldConfig;
typedef enum { RIPTIDE_LAND, RIPTIDE_WATER } RiptideSurface;
typedef struct {
    int64_t x, z;
    uint32_t variant; /**< Stable content-selection key; interpretation belongs to the game. */
    RiptideSurface surface;
    unsigned roads; /**< Bit 0:+Z, 1:+X, 2:-Z, 3:-X; reciprocal between dry neighbours. */
    float height;   /**< Height at the centre, metres. */
} RiptideCell;
typedef struct {
    int64_t x, z;
} RiptideOrigin;
/** Return a neutral procedural world configuration.
 * @param seed World seed.
 * @return Configuration with 64 m cells and low rolling relief. */
RiptideWorldConfig riptide_world_defaults(uint32_t seed);
/** Generate a cell without allocating or caching mutable state.
 * @param config World configuration, immutable during a streaming session.
 * @param x Global cell X in the supported range.
 * @param z Global cell Z in the supported range.
 * @param cell Output, unchanged on failure.
 * @return False for invalid coordinates or configuration.
 * Independent calls may run concurrently. No buildings or game assets are generated. */
bool riptide_world_cell(const RiptideWorldConfig *config, int64_t x, int64_t z, RiptideCell *cell);
/** Sample a continuous heightfield in global metres.
 * @param config World configuration.
 * @param x Global X in metres; keep within the supported cell range.
 * @param z Global Z in metres.
 * @return Bilinearly interpolated elevation, or zero for invalid arguments. */
float riptide_world_height(const RiptideWorldConfig *config, double x, double z);
/** Rebase a set of positions around a new global origin.
 * @param origin Current origin in cells; updated on success.
 * @param next New origin in cells.
 * @param cell_size Positive metres per cell.
 * @param positions Contiguous local-space positions; all world objects must be rebased together.
 * @param count Number of positions; NULL is allowed only when count is zero.
 * @return False for invalid arguments. Y and velocities are unchanged.
 * Rebase camera targets, particles and previous interpolation poses with the same delta. */
bool riptide_world_rebase(RiptideOrigin *origin, RiptideOrigin next, float cell_size, RiptideVec3 *positions,
                     size_t count);
typedef struct RiptideStream RiptideStream;
/** Observe a resident-cell transition.
 * @param user Application-owned context.
 * @param cell Borrowed descriptor, valid only during this call.
 * @param entering True to create content; false to release it.
 * Callbacks run synchronously on the caller thread. Do not reenter the stream. */
typedef void (*RiptideCellEventFn)(void *user, const RiptideCell *cell, bool entering);
/** Allocate a bounded square residency window.
 * @param config World configuration copied into the stream.
 * @param radius Half-width in cells, [0, 64]; capacity is (2*radius+1)^2.
 * @param event Optional residency callback.
 * @param user Context retained until destruction; caller owns it.
 * @return Owned stream or NULL on invalid input/allocation failure. */
RiptideStream *riptide_stream_create(RiptideWorldConfig config, unsigned radius, RiptideCellEventFn event, void *user);
/** Move the residency window and generate up to budget new cells.
 * @param stream Owned stream.
 * @param x Centre global cell X.
 * @param z Centre global cell Z.
 * @param budget Maximum entering callbacks this call; zero only unloads stale cells.
 * @return Number of newly resident cells. Repeat until riptide_stream_pending returns zero.
 * Leaving callbacks are not budgeted. Only one thread may access a stream at a time. */
unsigned riptide_stream_update(RiptideStream *stream, int64_t x, int64_t z, unsigned budget);
/** Count cells still missing from the most recently requested window.
 * @param stream Stream; may be NULL.
 * @return Pending count, or zero for NULL. */
unsigned riptide_stream_pending(const RiptideStream *stream);
/** Release resident content and stream memory.
 * @param stream Stream to destroy; NULL is accepted.
 * Calls the leaving callback for every resident cell before releasing storage. */
void riptide_stream_destroy(RiptideStream *stream);
#ifdef __cplusplus
}
#endif
#endif
