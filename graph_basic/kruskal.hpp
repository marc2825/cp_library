#pragma once
#include <algorithm>
#include <cassert>
#include <numeric>
#include <vector>
#include "spanning_forest.hpp"
#include "../graph/detail/distance.hpp"
#include "../union_find/disjoint_set_union.hpp"

// Docs: docs/graph_basic/kruskal.md
namespace cp {
// 無向グラフ限定。非連結なら各成分の最小全域木を返す。O(V+E log(E+1))。
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

} // namespace cp
