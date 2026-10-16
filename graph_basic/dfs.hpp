#pragma once
#include <cassert>
#include <cstddef>
#include <utility>
#include <vector>
#include "graph.hpp"
#include "shortest_path.hpp"

// Docs: docs/graph_basic/dfs.md
namespace cp {
struct DFSForestResult {
    std::vector<int> parent, parent_edge, depth, component;
    std::vector<int> roots, preorder, postorder, tin, tout;
};

// Visit all components. preorder[tin[v]..tout[v]) is the DFS-tree subtree of v.
// first_root=-1 visits roots in vertex order; otherwise its component is visited first.
template <class T>
DFSForestResult dfs_forest(const Graph<T>& g, int first_root = -1) {
    assert(first_root == -1 || (0 <= first_root && first_root < g.size()));
    DFSForestResult result;
    int n = g.size();
    result.parent.assign(n, -1); result.parent_edge.assign(n, -1);
    result.depth.assign(n, 0); result.component.assign(n, -1);
    result.tin.assign(n, -1); result.tout.assign(n, -1);
    auto visit = [&](int root) {
        if (result.component[root] >= 0) return;
        int id = static_cast<int>(result.roots.size());
        result.roots.push_back(root);
        result.component[root] = id;
        result.tin[root] = static_cast<int>(result.preorder.size());
        result.preorder.push_back(root);
        std::vector<std::pair<int, std::size_t>> stack{{root, 0}};
        while (!stack.empty()) {
            int v = stack.back().first;
            auto& next = stack.back().second;
            if (next == g[v].size()) {
                result.tout[v] = static_cast<int>(result.preorder.size());
                result.postorder.push_back(v); stack.pop_back(); continue;
            }
            const auto e = g[v][next++];
            if (result.component[e.to] >= 0) continue;
            result.parent[e.to] = v; result.parent_edge[e.to] = e.idx;
            result.depth[e.to] = result.depth[v] + 1; result.component[e.to] = id;
            result.tin[e.to] = static_cast<int>(result.preorder.size());
            result.preorder.push_back(e.to); stack.emplace_back(e.to, 0);
        }
    };
    if (first_root >= 0) visit(first_root);
    for (int v = 0; v < n; ++v) visit(v);
    return result;
}

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
