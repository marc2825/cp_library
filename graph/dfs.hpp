#pragma once
#include <cassert>
#include <cstddef>
#include <utility>
#include <vector>
#include "graph.hpp"
#include "shortest_path.hpp"

namespace cp {
// O(V+E)。再帰を使わない DFS。dist は DFS 木の深さであり最短距離ではない。
template <class T>
ShortestPathResult<int> depth_first_search(const Graph<T>& g, int start = 0) {
    assert(0 <= start && start < g.size());
    ShortestPathResult<int> result(g.size(), start);
    std::vector<std::pair<int, std::size_t>> stack{{start, 0}};
    result.dist[start] = 0;
    while (!stack.empty()) {
        int v = stack.back().first;
        auto& next = stack.back().second;
        if (next == g[v].size()) { stack.pop_back(); continue; }
        auto e = g[v][next++];
        if (result.reachable(e.to)) continue;
        result.dist[e.to] = result.dist[v] + 1;
        result.parent[e.to] = v; result.parent_edge[e.to] = e.idx;
        stack.emplace_back(e.to, 0);
    }
    return result;
}
} // namespace cp
