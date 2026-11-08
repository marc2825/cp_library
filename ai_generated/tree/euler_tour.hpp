#pragma once

#include <cstddef>
#include <utility>
#include <vector>
#include "rooted_tree.hpp"

namespace cp {

// O(N)。部分木クエリには tree.preorder と tree.subtree_interval(v) を使う。
// walk は子から戻るたびに親を追加する頂点列。edge_walk は各辺を往復で2回記録する。
// 詳細: docs/tree/euler_tour.md。
template <class T = long long>
struct EulerTour {
    RootedTree<T> tree;
    std::vector<int> walk, first, last;
    std::vector<int> edge_walk, edge_in, edge_out;

    explicit EulerTour(const Graph<T>& g, int root = 0) : tree(g, root) {
        first.resize(g.size()); last.resize(g.size());
        edge_in.resize(g.edge_count()); edge_out.resize(g.edge_count());
        for (int start : tree.roots) {
            first[start] = static_cast<int>(walk.size()); walk.push_back(start);
            std::vector<std::pair<int, std::size_t>> stack{{start, 0}};
            while (!stack.empty()) {
                int v = stack.back().first;
                auto& next = stack.back().second;
                if (next < g[v].size()) {
                    auto e = g[v][next++];
                    if (e.idx == tree.parent_edge[v]) continue;
                    edge_in[e.idx] = static_cast<int>(edge_walk.size()); edge_walk.push_back(e.idx);
                    first[e.to] = static_cast<int>(walk.size()); walk.push_back(e.to);
                    stack.emplace_back(e.to, 0);
                } else {
                    last[v] = static_cast<int>(walk.size()) - 1; stack.pop_back();
                    int p = tree.parent[v];
                    if (p == -1) continue;
                    int idx = tree.parent_edge[v];
                    edge_out[idx] = static_cast<int>(edge_walk.size()); edge_walk.push_back(idx);
                    walk.push_back(p);
                }
            }
        }
    }
};

} // namespace cp
