#pragma once

#include <cassert>
#include <queue>
#include "graph.hpp"
#include "shortest_path.hpp"

// Docs: docs/graph_basic/bellman_ford.md
namespace cp {

// O(V+VE)。negative_cycle[v] は始点から到達可能な負閉路の影響を受けるか。
template <class T>
ShortestPathResult<T> bellman_ford(const Graph<T>& g, int start = 0) {
    assert(0 <= start && start < g.size());
    ShortestPathResult<T> result(g.size(), start);
    result.dist[start] = T(0);
    auto relax = [&](int from, int to, T cost, int id) {
        if (!result.reachable(from)) return false;
        T next = detail::distance_add(result.dist[from], cost);
        if (next >= result.dist[to]) return false;
        result.dist[to] = next;
        result.parent[to] = from;
        result.parent_edge[to] = id;
        return true;
    };
    for (int iteration = 1; iteration < g.size(); ++iteration) {
        bool changed = false;
        for (const auto& e : g.edges()) {
            changed |= relax(e.from, e.to, e.cost, e.idx);
            if (!g.directed()) changed |= relax(e.to, e.from, e.cost, e.idx);
        }
        if (!changed) return result;
    }
    std::queue<int> queue;
    auto seed = [&](int from, int to, T cost) {
        if (result.reachable(from) && detail::distance_add(result.dist[from], cost) < result.dist[to]
            && !result.negative_cycle[to]) {
            result.negative_cycle[to] = true;
            queue.push(to);
        }
    };
    for (const auto& e : g.edges()) {
        seed(e.from, e.to, e.cost);
        if (!g.directed()) seed(e.to, e.from, e.cost);
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
