#pragma once
// 説明: docs/tree/rerooted_subtree.md
#include <stdexcept>
#include <utility>
#include <vector>
#include "heavy_light_decomposition.hpp"

namespace cp {
// hld.pos上の頂点区間（昇順、空区間なし）。異なる成分は拒否。O(log n)。
template<class T>
std::vector<std::pair<int, int>> rerooted_subtree_intervals(
        const HeavyLightDecomposition<T>& hld, int root, int v) {
    if (!hld.tree.same_component(root, v))
        throw std::invalid_argument("rerooted subtree vertices are disconnected");
    int fixed_root = hld.tree.roots[hld.tree.component[v]];
    auto whole = hld.subtree_interval(fixed_root);
    if (root == v) return {whole};
    if (!hld.tree.is_ancestor(v, root)) return {hld.subtree_interval(v)};
    int child = root;
    while (hld.head[child] != hld.head[v]) {
        int head = hld.head[child];
        if (hld.tree.parent[head] == v) { child = head; break; }
        child = hld.tree.parent[head];
    }
    if (hld.head[child] == hld.head[v]) child = hld.vertex_at[hld.pos[v] + 1];
    auto excluded = hld.subtree_interval(child);
    std::vector<std::pair<int, int>> result;
    if (whole.first < excluded.first) result.emplace_back(whole.first, excluded.first);
    if (excluded.second < whole.second) result.emplace_back(excluded.second, whole.second);
    return result;
}
} // namespace cp
