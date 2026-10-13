#pragma once
#include <algorithm>
#include <cassert>
#include <cstddef>
#include <utility>
#include <vector>
#include "graph.hpp"

namespace cp {
struct LowLinkResult {
    std::vector<int> ord, low, parent, parent_edge, subtree_size;
    std::vector<int> articulation, bridge_edges;
};
// 無向グラフ限定。非連結・自己ループ・多重辺に対応。再帰なし、O(V+E)。
template <class T>
LowLinkResult low_link(const Graph<T>& g) {
    assert(!g.directed());
    LowLinkResult result;
    result.ord.assign(g.size(), -1); result.low.resize(g.size());
    result.parent.assign(g.size(), -1); result.parent_edge.assign(g.size(), -1);
    result.subtree_size.assign(g.size(), 1);
    std::vector<int> children(g.size(), 0);
    std::vector<bool> articulation(g.size(), false), bridge(g.edge_count(), false);
    int timer = 0;
    for (int root = 0; root < g.size(); ++root) {
        if (result.ord[root] != -1) continue;
        result.ord[root] = result.low[root] = timer++;
        std::vector<std::pair<int, std::size_t>> stack{{root, 0}};
        while (!stack.empty()) {
            int v = stack.back().first;
            auto& next = stack.back().second;
            if (next < g[v].size()) {
                auto e = g[v][next++];
                if (e.idx == result.parent_edge[v]) continue;
                if (result.ord[e.to] == -1) {
                    ++children[v]; result.parent[e.to] = v; result.parent_edge[e.to] = e.idx;
                    result.ord[e.to] = result.low[e.to] = timer++;
                    stack.emplace_back(e.to, 0);
                } else result.low[v] = std::min(result.low[v], result.ord[e.to]);
            } else {
                stack.pop_back();
                int p = result.parent[v];
                if (p == -1) { articulation[v] = children[v] > 1; continue; }
                result.subtree_size[p] += result.subtree_size[v];
                result.low[p] = std::min(result.low[p], result.low[v]);
                if (result.parent[p] != -1 && result.ord[p] <= result.low[v]) articulation[p] = true;
                if (result.ord[p] < result.low[v]) bridge[result.parent_edge[v]] = true;
            }
        }
    }
    for (int v = 0; v < g.size(); ++v) if (articulation[v]) result.articulation.push_back(v);
    for (int e = 0; e < g.edge_count(); ++e) if (bridge[e]) result.bridge_edges.push_back(e);
    return result;
}
} // namespace cp
