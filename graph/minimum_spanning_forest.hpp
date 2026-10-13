#pragma once
#include <algorithm>
#include <cassert>
#include <functional>
#include <numeric>
#include <queue>
#include <utility>
#include <vector>
#include "graph.hpp"
#include "detail/distance.hpp"
#include "../data_structure/disjoint_set_union.hpp"

namespace cp {
template <class T>
struct MinimumSpanningForestResult {
    T cost = T(0);
    std::vector<Edge<T>> edges;
    int component_count = 0;
    bool connected() const { return component_count <= 1; }
};

// 無向グラフ限定。非連結なら各成分の最小全域木を返す。O(E log E)。
template <class T>
MinimumSpanningForestResult<T> kruskal(const Graph<T>& g) {
    assert(!g.directed());
    std::vector<int> order(g.edge_count());
    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(), [&](int a, int b) {
        if (g.edges()[a].cost != g.edges()[b].cost)
            return g.edges()[a].cost < g.edges()[b].cost;
        return a < b;
    });
    DisjointSetUnion dsu(g.size());
    MinimumSpanningForestResult<T> result;
    result.component_count = g.size();
    for (int idx : order) {
        const auto& e = g.edges()[idx];
        if (!dsu.merge(e.from, e.to)) continue;
        result.cost = detail::distance_add(result.cost, e.cost);
        result.edges.push_back(e); --result.component_count;
    }
    return result;
}

// 無向グラフ限定。自己ループ・多重辺・負辺・非連結に対応。O((V+E) log(V+E))。
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
