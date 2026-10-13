#pragma once
#include <cassert>
#include <optional>
#include <queue>
#include <vector>
#include "graph.hpp"

namespace cp {
// 有向グラフ限定。閉路があれば nullopt。空の DAG は空の配列。O(V+E)。
template <class T>
std::optional<std::vector<int>> topological_sort(const Graph<T>& g) {
    assert(g.directed());
    std::vector<int> indegree(g.size(), 0), order;
    for (const auto& e : g.edges()) ++indegree[e.to];
    std::queue<int> queue;
    for (int v = 0; v < g.size(); ++v) if (indegree[v] == 0) queue.push(v);
    while (!queue.empty()) {
        int v = queue.front(); queue.pop(); order.push_back(v);
        for (const auto& e : g[v]) if (--indegree[e.to] == 0) queue.push(e.to);
    }
    if (static_cast<int>(order.size()) != g.size()) return std::nullopt;
    return order;
}
} // namespace cp
