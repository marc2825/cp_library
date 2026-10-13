#pragma once

#include <algorithm>
#include <cassert>
#include <vector>
#include "detail/distance.hpp"

namespace cp {

template <class T>
struct ShortestPathResult {
    int source = -1;
    std::vector<T> dist;
    std::vector<int> parent, parent_edge;
    // 負閉路から到達可能な頂点。その頂点の dist は最短距離として使用できない。
    std::vector<bool> negative_cycle;

    explicit ShortestPathResult(int n = 0, int start = -1)
        : source(start), dist(n, distance_inf<T>()), parent(n, -1),
          parent_edge(n, -1), negative_cycle(n, false) {}

    bool reachable(int v) const {
        assert(0 <= v && v < static_cast<int>(dist.size()));
        return dist[v] != distance_inf<T>();
    }
    bool has_finite_distance(int v) const {
        return reachable(v) && !negative_cycle[v];
    }

    // 到達不能・負閉路の影響下では空列。始点自身への頂点列は {source}。
    std::vector<int> vertex_path(int target) const {
        if (!has_finite_distance(target)) return {};
        std::vector<int> result;
        for (int v = target; v != -1; v = parent[v]) {
            result.push_back(v);
            if (v == source) break;
            assert(result.size() <= dist.size());
        }
        std::reverse(result.begin(), result.end());
        return result;
    }

    std::vector<int> edge_path(int target) const {
        if (!has_finite_distance(target)) return {};
        std::vector<int> result;
        for (int v = target; v != source; v = parent[v]) {
            assert(v != -1 && parent_edge[v] != -1);
            result.push_back(parent_edge[v]);
            assert(result.size() <= dist.size());
        }
        std::reverse(result.begin(), result.end());
        return result;
    }
};

} // namespace cp
