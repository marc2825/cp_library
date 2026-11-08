#pragma once
#include <vector>
#include "graph.hpp"
#include "low_link.hpp"

namespace cp {
template <class T>
struct TwoEdgeConnectedComponentsResult {
    std::vector<int> component;
    std::vector<std::vector<int>> groups;
    Graph<T> forest;
    // forest の辺番号 -> 元のグラフの辺番号。
    std::vector<int> original_edge;
};
// 橋を除いた連結成分と、その縮約後の森。O(V+E)。
template <class T>
TwoEdgeConnectedComponentsResult<T> two_edge_connected_components(const Graph<T>& g) {
    auto low = low_link(g);
    std::vector<bool> bridge(g.edge_count(), false);
    for (int idx : low.bridge_edges) bridge[idx] = true;
    TwoEdgeConnectedComponentsResult<T> result;
    result.component.assign(g.size(), -1);
    for (int root = 0; root < g.size(); ++root) {
        if (result.component[root] != -1) continue;
        int id = static_cast<int>(result.groups.size());
        result.groups.push_back({}); result.component[root] = id;
        std::vector<int> stack{root};
        while (!stack.empty()) {
            int v = stack.back(); stack.pop_back(); result.groups.back().push_back(v);
            for (const auto& e : g[v]) {
                if (bridge[e.idx] || result.component[e.to] != -1) continue;
                result.component[e.to] = id; stack.push_back(e.to);
            }
        }
    }
    result.forest = Graph<T>(static_cast<int>(result.groups.size()));
    for (int idx : low.bridge_edges) {
        const auto& e = g.edges()[idx];
        result.forest.add_edge(result.component[e.from], result.component[e.to], e.cost);
        result.original_edge.push_back(idx);
    }
    return result;
}
} // namespace cp
