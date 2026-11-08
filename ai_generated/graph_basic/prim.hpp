#pragma once
#include <cassert>
#include <functional>
#include <queue>
#include <utility>
#include <vector>
#include "spanning_forest.hpp"
#include "../graph/detail/distance.hpp"

// Docs: docs/graph_basic/prim.md
namespace cp {
// 無向グラフ限定。自己ループ・多重辺・負辺・非連結に対応。O(V+E log(E+1))。
template <class T>
MinimumSpanningForestResult<T> prim(const Graph<T>& g) {
    assert(!g.directed());
    MinimumSpanningForestResult<T> result;
    std::vector<bool> used(g.size(), false);
    using Entry = std::pair<T, int>;
    std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> queue;
    auto visit = [&](int v) {
        used[v] = true;
        for (const auto& e : g[v]) if (!used[e.to]) queue.emplace(e.cost, e.idx);
    };
    for (int root = 0; root < g.size(); ++root) {
        if (used[root]) continue;
        ++result.component_count; visit(root);
        while (!queue.empty()) {
            auto [cost, idx] = queue.top(); queue.pop();
            const auto& e = g.edges()[idx];
            if (used[e.from] && used[e.to]) continue;
            result.cost = detail::distance_add(result.cost, cost);
            result.edges.push_back(e);
            visit(used[e.from] ? e.to : e.from);
        }
    }
    return result;
}
} // namespace cp
