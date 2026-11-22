#pragma once
#include <limits>
#include <vector>
#include "graph.hpp"
#include "shortest_path.hpp"
// Docs: docs/graph_basic/floyd_warshall.md
namespace cp {
// O(V^3) 時間 / O(V^2) 空間。result[s] は始点 s の結果。
// 負閉路を経由できる組は negative_cycle を立て、dist を最小値にする。その組では
// 加算をしないため、負閉路があっても値が指数的に小さくなってあふれることはない。
template <class T>
std::vector<ShortestPathResult<T>> floyd_warshall(const Graph<T>& g) {
    const int n = g.size();
    std::vector<ShortestPathResult<T>> result;
    for (int s = 0; s < n; ++s) {
        result.emplace_back(n, s);
        result[s].dist[s] = T(0);
        for (const auto& e : g[s]) {
            if (e.cost >= result[s].dist[e.to]) continue;
            result[s].dist[e.to] = e.cost;
            result[s].parent[e.to] = s;
            result[s].parent_edge[e.to] = e.idx;
        }
    }
    auto minus_infinity = [&](int s, int t) {
        result[s].negative_cycle[t] = true;
        result[s].dist[t] = std::numeric_limits<T>::lowest();
        result[s].parent[t] = result[s].parent_edge[t] = -1;
    };
    for (int k = 0; k < n; ++k) {
        // 中継点 < k の歩道で k に戻る負の閉路ができた: k を通る組はすべて −∞。
        const bool through_cycle = result[k].negative_cycle[k] || result[k].dist[k] < T(0);
        for (int s = 0; s < n; ++s) {
            if (!result[s].reachable(k)) continue;
            const bool left = through_cycle || result[s].negative_cycle[k];
            for (int t = 0; t < n; ++t) {
                if (!result[k].reachable(t) || result[s].negative_cycle[t]) continue;
                if (left || result[k].negative_cycle[t]) { minus_infinity(s, t); continue; }
                T next = detail::distance_add(result[s].dist[k], result[k].dist[t]);
                if (next >= result[s].dist[t]) continue;
                result[s].dist[t] = next;
                result[s].parent[t] = result[k].parent[t];
                result[s].parent_edge[t] = result[k].parent_edge[t];
            }
        }
    }
    return result;
}
} // namespace cp
