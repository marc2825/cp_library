#pragma once

#include <algorithm>
#include <cassert>
#include <utility>
#include <vector>
#include "rooted_tree.hpp"

namespace cp {

struct PathSegment {
    int l, r; // 半開区間 [l,r)。空区間は返さない。
    bool reversed; // true なら r-1, r-2, ..., l の順で処理する。
};

// 前計算 O(N)。部分木は1区間、パスは O(log N) 個の区間。有向の処理順も保持する。
template <class T = long long>
class HeavyLightDecomposition {
public:
    RootedTree<T> tree;
    std::vector<int> head, pos, vertex_at, heavy;

    explicit HeavyLightDecomposition(const Graph<T>& g, int root = 0) : tree(g, root) {
        head.resize(g.size()); pos.resize(g.size()); heavy.assign(g.size(), -1);
        for (int v : tree.preorder) {
            int p = tree.parent[v];
            if (p != -1 && (heavy[p] == -1 || tree.subtree_size[v] > tree.subtree_size[heavy[p]]))
                heavy[p] = v;
        }
        for (int start : tree.roots) {
            std::vector<std::pair<int, int>> stack{{start, start}};
            while (!stack.empty()) {
                auto [v, chain] = stack.back(); stack.pop_back();
                head[v] = chain; pos[v] = static_cast<int>(vertex_at.size()); vertex_at.push_back(v);
                for (auto it = g[v].rbegin(); it != g[v].rend(); ++it)
                    if (tree.parent[it->to] == v && it->to != heavy[v]) stack.emplace_back(it->to, it->to);
                if (heavy[v] != -1) stack.emplace_back(heavy[v], chain);
            }
        }
    }

    std::pair<int, int> subtree_interval(int v) const {
        assert(0 <= v && v < tree.size());
        return {pos[v], pos[v] + tree.subtree_size[v]};
    }
    int lca(int u, int v) const {
        if (!tree.same_component(u, v)) return -1;
        while (head[u] != head[v]) {
            if (tree.depth[head[u]] < tree.depth[head[v]]) std::swap(u, v);
            u = tree.parent[head[u]];
        }
        return tree.depth[u] < tree.depth[v] ? u : v;
    }
    // u -> v の順。edge_values=true なら辺の値を子の位置に置き、LCA の位置を除く。
    std::vector<PathSegment> path_segments(int u, int v, bool edge_values = false) const {
        assert(tree.same_component(u, v));
        std::vector<PathSegment> left, right;
        while (head[u] != head[v]) {
            if (tree.depth[head[u]] >= tree.depth[head[v]]) {
                left.push_back({pos[head[u]], pos[u] + 1, true}); u = tree.parent[head[u]];
            } else {
                right.push_back({pos[head[v]], pos[v] + 1, false}); v = tree.parent[head[v]];
            }
        }
        int l = std::min(pos[u], pos[v]) + (edge_values ? 1 : 0);
        int r = std::max(pos[u], pos[v]) + 1;
        if (l < r) left.push_back({l, r, pos[u] > pos[v]});
        left.insert(left.end(), right.rbegin(), right.rend());
        return left;
    }
};

} // namespace cp
