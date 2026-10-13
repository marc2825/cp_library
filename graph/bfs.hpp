#pragma once

#include <cassert>
#include <queue>
#include "graph.hpp"
#include "shortest_path.hpp"

namespace cp {

// O(V+E)。cost は使わず、辺の本数による最短距離を返す。
template <class T>
ShortestPathResult<int> bfs(const Graph<T>& g, int start = 0) {
    assert(0 <= start && start < g.size());
    ShortestPathResult<int> result(g.size(), start);
    std::queue<int> queue;
    result.dist[start] = 0;
    queue.push(start);
    while (!queue.empty()) {
        int v = queue.front();
        queue.pop();
        for (const auto& e : g[v]) {
            if (result.reachable(e.to)) continue;
            result.dist[e.to] = result.dist[v] + 1;
            result.parent[e.to] = v;
            result.parent_edge[e.to] = e.idx;
            queue.push(e.to);
        }
    }
    return result;
}

} // namespace cp
