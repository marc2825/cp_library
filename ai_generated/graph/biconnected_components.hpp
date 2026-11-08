#pragma once
#include <algorithm>
#include <cstddef>
#include <utility>
#include <vector>
#include "graph.hpp"
#include "low_link.hpp"

namespace cp {
// 各頂点二重連結成分の頂点集合。橋も2頂点のブロック、孤立点は単独ブロック。
// 自己ループは頂点連結性に影響しないため無視。再帰なし、O(V+E)。
template <class T>
std::vector<std::vector<int>> biconnected_components(const Graph<T>& g) {
    auto low = low_link(g);
    std::vector<std::vector<int>> groups;
    std::vector<int> edge_stack, seen(g.size(), -1);
    for (int root = 0; root < g.size(); ++root) {
        if (low.parent[root] != -1) continue;
        bool has_edge = false;
        std::vector<std::pair<int, std::size_t>> stack{{root, 0}};
        while (!stack.empty()) {
            int v = stack.back().first;
            auto& next = stack.back().second;
            if (next < g[v].size()) {
                auto e = g[v][next++];
                if (e.to == v || e.idx == low.parent_edge[v]) continue;
                has_edge = true;
                if (low.parent[e.to] == v && low.parent_edge[e.to] == e.idx) {
                    edge_stack.push_back(e.idx); stack.emplace_back(e.to, 0);
                } else if (low.ord[e.to] < low.ord[v]) edge_stack.push_back(e.idx);
            } else {
                stack.pop_back();
                int p = low.parent[v];
                if (p == -1 || low.low[v] < low.ord[p]) continue;
                int id = static_cast<int>(groups.size()); groups.push_back({});
                while (true) {
                    int idx = edge_stack.back(); edge_stack.pop_back();
                    const auto& e = g.edges()[idx];
                    for (int u : {e.from, e.to}) {
                        if (seen[u] == id) continue;
                        seen[u] = id; groups.back().push_back(u);
                    }
                    if (idx == low.parent_edge[v]) break;
                }
            }
        }
        if (!has_edge) groups.push_back({root});
    }
    return groups;
}

template <class T>
struct BlockCutForestResult {
    std::vector<std::vector<int>> blocks;
    // [0,n) は元の頂点、[n,n+blocks.size()) はブロック。各所属関係を辺とする森。
    Graph<int> forest;
};

template <class T>
BlockCutForestResult<T> block_cut_forest(const Graph<T>& g) {
    BlockCutForestResult<T> result;
    result.blocks = biconnected_components(g);
    result.forest = Graph<int>(g.size() + static_cast<int>(result.blocks.size()));
    for (int i = 0; i < static_cast<int>(result.blocks.size()); ++i)
        for (int v : result.blocks[i]) result.forest.add_edge(v, g.size() + i);
    return result;
}
} // namespace cp
