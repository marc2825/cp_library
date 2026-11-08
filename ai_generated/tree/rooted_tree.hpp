#pragma once

#include <algorithm>
#include <cassert>
#include <cstddef>
#include <stdexcept>
#include <utility>
#include <vector>
#include "../graph/graph.hpp"
#include "../graph/detail/distance.hpp"

namespace cp {

// 無向の木・森を根付きにする。O(V+E)。非再帰。閉路があれば invalid_argument。
// root の成分を先に処理し、残りの成分は頂点番号の小さい頂点を根とする。
// 構築時のスナップショット。元の Graph の変更後はオブジェクトを作り直す。
// 詳細: docs/tree/rooted_tree.md。
template <class T = long long>
struct RootedTree {
    std::vector<int> roots, parent, parent_edge, depth, subtree_size;
    std::vector<int> component, preorder, postorder, tin, tout;
    std::vector<T> distance_from_root;

    RootedTree() = default;
    explicit RootedTree(const Graph<T>& g, int root = 0) {
        if (g.directed()) throw std::invalid_argument("RootedTree requires an undirected forest");
        int n = g.size();
        parent.assign(n, -1); parent_edge.assign(n, -1); depth.assign(n, 0);
        subtree_size.assign(n, 1); component.assign(n, -1);
        distance_from_root.assign(n, T(0)); tin.resize(n); tout.resize(n);
        if (n == 0) return;
        assert(0 <= root && root < n);
        for (int i = -1; i < n; ++i) {
            int start = i == -1 ? root : i;
            if (component[start] != -1) continue;
            int id = static_cast<int>(roots.size()); roots.push_back(start);
            component[start] = id;
            tin[start] = static_cast<int>(preorder.size()); preorder.push_back(start);
            std::vector<std::pair<int, std::size_t>> stack{{start, 0}};
            while (!stack.empty()) {
                int v = stack.back().first;
                auto& next = stack.back().second;
                if (next < g[v].size()) {
                    auto e = g[v][next++];
                    if (e.idx == parent_edge[v]) continue;
                    if (component[e.to] != -1) throw std::invalid_argument("graph contains a cycle");
                    component[e.to] = id; parent[e.to] = v; parent_edge[e.to] = e.idx;
                    depth[e.to] = depth[v] + 1;
                    distance_from_root[e.to] = detail::distance_add(distance_from_root[v], e.cost);
                    tin[e.to] = static_cast<int>(preorder.size()); preorder.push_back(e.to);
                    stack.emplace_back(e.to, 0);
                } else {
                    postorder.push_back(v);
                    tout[v] = static_cast<int>(preorder.size()); stack.pop_back();
                    if (parent[v] != -1) subtree_size[parent[v]] += subtree_size[v];
                }
            }
        }
    }

    int size() const { return static_cast<int>(parent.size()); }
    bool same_component(int u, int v) const {
        assert(0 <= u && u < size() && 0 <= v && v < size());
        return component[u] == component[v];
    }
    bool is_ancestor(int u, int v) const {
        return same_component(u, v) && tin[u] <= tin[v] && tin[v] < tout[u];
    }
    // 部分木は preorder 上の半開区間 [tin[v], tout[v])。
    std::pair<int, int> subtree_interval(int v) const {
        assert(0 <= v && v < size());
        return {tin[v], tout[v]};
    }
};

} // namespace cp
