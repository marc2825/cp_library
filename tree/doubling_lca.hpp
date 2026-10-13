#pragma once

#include <algorithm>
#include <cassert>
#include <utility>
#include <vector>
#include "rooted_tree.hpp"

namespace cp {

// 前計算 O(N log N)、lca / distance / jump は O(log N)。森にも対応。
template <class T = long long>
class DoublingLCA {
public:
    RootedTree<T> tree;
    std::vector<std::vector<int>> up; // up[k][v] は 2^k 個上の祖先。存在しなければ -1。

    explicit DoublingLCA(const Graph<T>& g, int root = 0) : DoublingLCA(RootedTree<T>(g, root)) {}
    explicit DoublingLCA(RootedTree<T> rooted) : tree(std::move(rooted)) {
        int levels = 1;
        while ((1LL << levels) <= tree.size()) ++levels;
        up.assign(levels, std::vector<int>(tree.size(), -1));
        up[0] = tree.parent;
        for (int k = 1; k < levels; ++k)
            for (int v = 0; v < tree.size(); ++v)
                if (up[k-1][v] != -1) up[k][v] = up[k-1][up[k-1][v]];
    }

    int kth_ancestor(int v, long long k) const {
        assert(0 <= v && v < tree.size() && k >= 0);
        if (k > tree.depth[v]) return -1;
        for (int bit = 0; k; ++bit, k >>= 1) if (k & 1) v = up[bit][v];
        return v;
    }
    int lca(int u, int v) const {
        if (!tree.same_component(u, v)) return -1;
        if (tree.depth[u] < tree.depth[v]) std::swap(u, v);
        u = kth_ancestor(u, tree.depth[u] - tree.depth[v]);
        if (u == v) return u;
        for (int k = static_cast<int>(up.size()) - 1; k >= 0; --k)
            if (up[k][u] != up[k][v]) { u = up[k][u]; v = up[k][v]; }
        return tree.parent[u];
    }
    int edge_distance(int u, int v) const {
        int w = lca(u, v); assert(w != -1);
        return (tree.depth[u] - tree.depth[w]) + (tree.depth[v] - tree.depth[w]);
    }
    T distance(int u, int v) const {
        int w = lca(u, v); assert(w != -1);
        return detail::distance_add(
            detail::distance_subtract(tree.distance_from_root[u], tree.distance_from_root[w]),
            detail::distance_subtract(tree.distance_from_root[v], tree.distance_from_root[w]));
    }
    bool is_on_path(int s, int t, int v) const {
        if (!tree.same_component(s, t) || !tree.same_component(s, v)) return false;
        return edge_distance(s, v) + edge_distance(v, t) == edge_distance(s, t);
    }
    // u -> v のパスの k 番目の頂点 (0-indexed)。範囲外・別成分なら -1。
    int jump(int u, int v, long long k) const {
        assert(k >= 0);
        int w = lca(u, v);
        if (w == -1) return -1;
        int a = tree.depth[u] - tree.depth[w], b = tree.depth[v] - tree.depth[w];
        if (k > static_cast<long long>(a) + b) return -1;
        return k <= a ? kth_ancestor(u, k) : kth_ancestor(v, a + b - k);
    }
    std::vector<int> vertex_path(int u, int v) const {
        int w = lca(u, v);
        if (w == -1) return {};
        std::vector<int> left, right;
        for (; u != w; u = tree.parent[u]) left.push_back(u);
        left.push_back(w);
        for (; v != w; v = tree.parent[v]) right.push_back(v);
        left.insert(left.end(), right.rbegin(), right.rend());
        return left;
    }
};

} // namespace cp
