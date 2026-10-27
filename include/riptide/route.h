/* SPDX-License-Identifier: MIT */
#ifndef RIPTIDE_ROUTE_H
#define RIPTIDE_ROUTE_H
#include "core.h"
#ifdef __cplusplus
extern "C" {
#endif
#define RIPTIDE_ROUTE_NONE UINT32_MAX
typedef struct {
    uint32_t to;
    float cost;
} RiptideRoadEdge;
typedef struct {
    RiptideVec3 position;
    uint32_t first_edge, edge_count;
} RiptideRoadNode;
typedef struct {
    const RiptideRoadNode *nodes;
    size_t node_count;
    const RiptideRoadEdge *edges;
    size_t edge_count;
} RiptideRoadGraph;
typedef struct {
    float distance;
    uint32_t previous;
    bool visited;
} RiptideRouteWork;
/** Find the minimum-cost route through a directed road graph.
 * @param graph Borrowed node/edge arrays; costs must be finite and nonnegative.
 * @param start Start node index.
 * @param goal Destination node index.
 * @param work Caller scratch array with at least graph->node_count entries.
 * @param work_count Scratch capacity; no allocations occur in this function.
 * @param path Output node indices from start through goal.
 * @param capacity Path capacity in nodes.
 * @return Required node count, including endpoints; zero for invalid/unreachable input.
 * If the result exceeds capacity, path is unchanged. NULL path with zero capacity
 * queries the required size. Dijkstra search is O(V^2+E); use regional graphs. */
size_t riptide_route_find(const RiptideRoadGraph *graph, uint32_t start, uint32_t goal, RiptideRouteWork *work,
                     size_t work_count, uint32_t *path, size_t capacity);
/** Steer toward a waypoint using the same heading convention as vehicle_step.
 * @param position Vehicle ground position.
 * @param yaw Current heading in radians.
 * @param target Waypoint position.
 * @param gain Positive steering response per radian.
 * @return Steering input clamped to [-1, 1]; zero if already at the target. */
float riptide_route_steering(RiptideVec3 position, float yaw, RiptideVec3 target, float gain);
#ifdef __cplusplus
}
#endif
#endif
