#pragma once

#include <cassert>
#include <functional>
#include <queue>
#include <utility>
#include "graph.hpp"
#include "shortest_path.hpp"

namespace cp {

// 非負重み限定。自己ループ・多重辺に対応。O((V+E) log(V+E))。
template <class T>
ShortestPathResult<T> dijkstra(const Graph<T>& g, int start = 0) {
    assert(0 <= start && start < g.size());
    for (const auto& e : g.edges()) { assert(e.cost >= T(0)); (void)e; }
    ShortestPathResult<T> result(g.size(), start);
    using Entry = std::pair<T, int>;
    std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> queue;
    result.dist[start] = T(0);
    queue.emplace(T(0), start);
    while (!queue.empty()) {
        auto [distance, v] = queue.top();
        queue.pop();
        if (distance != result.dist[v]) continue;
        for (const auto& e : g[v]) {
            T next = detail::distance_add(distance, e.cost);
            if (next >= result.dist[e.to]) continue;
            result.dist[e.to] = next;
            result.parent[e.to] = v;
            result.parent_edge[e.to] = e.idx;
            queue.emplace(next, e.to);
        }
    }
    return result;
}

} // namespace cp
