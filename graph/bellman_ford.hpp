#pragma once

#include <cassert>
#include <queue>
#include "graph.hpp"
#include "shortest_path.hpp"

namespace cp {

// O(VE)。negative_cycle[v] は始点から到達可能な負閉路の影響を受けるか。
template <class T>
ShortestPathResult<T> bellman_ford(const Graph<T>& g, int start = 0) {
    assert(0 <= start && start < g.size());
    ShortestPathResult<T> result(g.size(), start);
    result.dist[start] = T(0);
    for (int iteration = 1; iteration < g.size(); ++iteration) {
        bool changed = false;
        for (int v = 0; v < g.size(); ++v) {
            if (!result.reachable(v)) continue;
            for (const auto& e : g[v]) {
                T next = detail::distance_add(result.dist[v], e.cost);
                if (next >= result.dist[e.to]) continue;
                result.dist[e.to] = next;
                result.parent[e.to] = v;
                result.parent_edge[e.to] = e.idx;
                changed = true;
            }
        }
        if (!changed) return result;
    }
    std::queue<int> queue;
    for (int v = 0; v < g.size(); ++v) {
        if (!result.reachable(v)) continue;
        for (const auto& e : g[v]) {
            if (detail::distance_add(result.dist[v], e.cost) < result.dist[e.to]
                && !result.negative_cycle[e.to]) {
                result.negative_cycle[e.to] = true;
                queue.push(e.to);
            }
        }
    }
    while (!queue.empty()) {
        int v = queue.front();
        queue.pop();
        for (const auto& e : g[v]) {
            if (result.negative_cycle[e.to]) continue;
            result.negative_cycle[e.to] = true;
            queue.push(e.to);
        }
    }
    return result;
}

} // namespace cp
