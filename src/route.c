/* SPDX-License-Identifier: MIT */
#include "riptide/route.h"
#include <math.h>
#include <float.h>
size_t riptide_route_find(const RiptideRoadGraph *g, uint32_t start, uint32_t goal, RiptideRouteWork *w,
                     size_t count, uint32_t *path, size_t capacity) {
    if (!g || !g->nodes || (!g->edges && g->edge_count) || !w || count < g->node_count ||
        g->node_count > UINT32_MAX || start >= g->node_count || goal >= g->node_count ||
        (!path && capacity))
        return 0;
    for (size_t i = 0; i < g->node_count; i++) {
        const RiptideRoadNode *n = &g->nodes[i];
        if (n->first_edge > g->edge_count || n->edge_count > g->edge_count - n->first_edge)
            return 0;
        w[i] = (RiptideRouteWork){FLT_MAX, RIPTIDE_ROUTE_NONE, false};
    }
    for (size_t i = 0; i < g->edge_count; i++)
        if (g->edges[i].to >= g->node_count || !isfinite(g->edges[i].cost) || g->edges[i].cost < 0)
            return 0;
    w[start].distance = 0;
    for (size_t pass = 0; pass < g->node_count; pass++) {
        uint32_t best = RIPTIDE_ROUTE_NONE;
        float nearest = FLT_MAX;
        for (size_t i = 0; i < g->node_count; i++)
            if (!w[i].visited && w[i].distance < nearest) {
                best = (uint32_t)i;
                nearest = w[i].distance;
            }
        if (best == RIPTIDE_ROUTE_NONE)
            return 0;
        if (best == goal)
            break;
        w[best].visited = true;
        const RiptideRoadNode *n = &g->nodes[best];
        for (uint32_t j = 0; j < n->edge_count; j++) {
            RiptideRoadEdge e = g->edges[n->first_edge + j];
            float cost = nearest + e.cost;
            if (!w[e.to].visited && cost < w[e.to].distance) {
                w[e.to].distance = cost;
                w[e.to].previous = best;
            }
        }
    }
    size_t needed = 1;
    uint32_t at = goal;
    while (at != start) {
        at = w[at].previous;
        if (at == RIPTIDE_ROUTE_NONE || needed >= g->node_count)
            return 0;
        needed++;
    }
    if (capacity >= needed && path) {
        at = goal;
        for (size_t i = needed; i > 0; i--) {
            path[i - 1] = at;
            at = w[at].previous;
        }
    }
    return needed;
}
float riptide_route_steering(RiptideVec3 p, float yaw, RiptideVec3 target, float gain) {
    float dx = target.x - p.x, dz = target.z - p.z;
    if (hypotf(dx, dz) < .001f)
        return 0;
    return riptide_clamp(riptide_angle_delta(atan2f(dx, dz), yaw) * gain, -1, 1);
}
