#pragma once

#include <vector>
#include "graph.hpp"
#include "shortest_path.hpp"

// Docs: docs/graph_basic/floyd_warshall.md
namespace cp {

// O(V^3) 時間 / O(V^2) 空間。result[s] は始点 s の結果。
template <class T>
std::vector<ShortestPathResult<T>> floyd_warshall(const Graph<T>& g) {
    std::vector<ShortestPathResult<T>> result;
    for (int s = 0; s < g.size(); ++s) {
        result.emplace_back(g.size(), s);
        result[s].dist[s] = T(0);
        for (const auto& e : g[s]) {
            if (e.cost >= result[s].dist[e.to]) continue;
            result[s].dist[e.to] = e.cost;
            result[s].parent[e.to] = s;
            result[s].parent_edge[e.to] = e.idx;
        }
    }
    for (int k = 0; k < g.size(); ++k) {
        for (int s = 0; s < g.size(); ++s) {
            if (!result[s].reachable(k)) continue;
            for (int t = 0; t < g.size(); ++t) {
                if (!result[k].reachable(t)) continue;
                T next = detail::distance_add(result[s].dist[k], result[k].dist[t]);
                if (next >= result[s].dist[t]) continue;
                result[s].dist[t] = next;
                result[s].parent[t] = result[k].parent[t];
                result[s].parent_edge[t] = result[k].parent_edge[t];
            }
        }
    }
    for (int k = 0; k < g.size(); ++k) {
        if (result[k].dist[k] >= T(0)) continue;
        for (int s = 0; s < g.size(); ++s) {
            if (!result[s].reachable(k)) continue;
            for (int t = 0; t < g.size(); ++t)
                if (result[k].reachable(t)) result[s].negative_cycle[t] = true;
        }
    }
    return result;
}

} // namespace cp
