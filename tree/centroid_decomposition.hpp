#pragma once

#include <algorithm>
#include <utility>
#include <vector>
#include "rooted_tree.hpp"

namespace cp {

// 各連結成分の重心を列挙する (各成分1個または2個)。O(N)。重みは無視。
template <class T>
std::vector<int> tree_centroids(const Graph<T>& g) {
    RootedTree<T> tree(g);
    std::vector<int> result;
    for (int v = 0; v < g.size(); ++v) {
        int total = tree.subtree_size[tree.roots[tree.component[v]]];
        int largest = total - tree.subtree_size[v];
        for (const auto& e : g[v])
            if (tree.parent[e.to] == v) largest = std::max(largest, tree.subtree_size[e.to]);
        if (largest <= total / 2) result.push_back(v);
    }
    return result;
}

// O(N log N)。非再帰。parent は重心分解木の親、level はその深さ。
template <class T = long long>
struct CentroidDecomposition {
    std::vector<int> parent, level, component_size, roots;

    explicit CentroidDecomposition(const Graph<T>& g) {
        RootedTree<T> tree(g); // 森であることを検証。
        parent.assign(g.size(), -1); level.assign(g.size(), 0); component_size.resize(g.size());
        std::vector<bool> removed(g.size(), false);
        std::vector<int> local_parent(g.size(), -1), size(g.size());
        std::vector<std::pair<int, int>> tasks;
        for (int root : tree.roots) tasks.emplace_back(root, -1);
        while (!tasks.empty()) {
            auto [start, previous] = tasks.back(); tasks.pop_back();
            std::vector<int> order{start}; local_parent[start] = -1;
            for (std::size_t i = 0; i < order.size(); ++i) {
                int v = order[i]; size[v] = 1;
                for (const auto& e : g[v]) {
                    if (removed[e.to] || e.to == local_parent[v]) continue;
                    local_parent[e.to] = v; order.push_back(e.to);
                }
            }
            for (auto it = order.rbegin(); it != order.rend(); ++it)
                if (local_parent[*it] != -1) size[local_parent[*it]] += size[*it];
            int total = static_cast<int>(order.size()), center = start;
            for (int v : order) {
                int largest = total - size[v];
                for (const auto& e : g[v])
                    if (!removed[e.to] && local_parent[e.to] == v) largest = std::max(largest, size[e.to]);
                if (largest <= total / 2) { center = v; break; }
            }
            parent[center] = previous;
            level[center] = previous == -1 ? 0 : level[previous] + 1;
            component_size[center] = total;
            if (previous == -1) roots.push_back(center);
            removed[center] = true;
            for (const auto& e : g[center]) if (!removed[e.to]) tasks.emplace_back(e.to, center);
        }
    }
};

} // namespace cp
