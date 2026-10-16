#pragma once

#include <cassert>
#include <deque>
#include <utility>
#include "graph.hpp"
#include "shortest_path.hpp"

// Docs: docs/graph_basic/zero_one_bfs.md
namespace cp {

// cost が 0 または 1 のグラフ限定。O(V+E)。
template <class T>
ShortestPathResult<T> zero_one_bfs(const Graph<T>& g, int start = 0) {
    assert(0 <= start && start < g.size());
    for (const auto& e : g.edges()) {
        assert(e.cost == T(0) || e.cost == T(1)); (void)e;
    }
    ShortestPathResult<T> result(g.size(), start);
    std::deque<std::pair<T, int>> queue;
    result.dist[start] = T(0);
    queue.emplace_front(T(0), start);
    while (!queue.empty()) {
        auto [distance, v] = queue.front();
        queue.pop_front();
        if (distance != result.dist[v]) continue;
        for (const auto& e : g[v]) {
            T next = detail::distance_add(distance, e.cost);
            if (next >= result.dist[e.to]) continue;
            result.dist[e.to] = next;
            result.parent[e.to] = v;
            result.parent_edge[e.to] = e.idx;
            if (e.cost == T(0)) queue.emplace_front(next, e.to);
            else queue.emplace_back(next, e.to);
        }
    }
    return result;
}

} // namespace cp
