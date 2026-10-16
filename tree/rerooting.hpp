#pragma once

#include <vector>
#include "tree_dp.hpp"

namespace cp {

// 全頂点を根とした DP。結合的 merge と単位元が必要、可換性・逆元は不要。
// 各頂点の隣接リスト順を保持。戻り値は vector<R>。詳細: docs/tree/rerooting.md。
template <class S, class T, class Merge, class AddVertex, class Lift>
auto rerooting(const Graph<T>& g, const S& identity, Merge merge,
               AddVertex add_vertex, Lift lift, int root = 0) {
    RootedTree<T> tree(g, root);
    auto down = detail::tree_dp_bottom_up(g, tree, identity, merge, add_vertex, lift);
    auto answer = down;
    std::vector<S> from_parent(g.size(), identity);
    for (int v : tree.preorder) {
        const int degree = static_cast<int>(g[v].size());
        std::vector<S> incoming(degree, identity), prefix(degree + 1, identity);
        for (int i = 0; i < degree; ++i) {
            const auto& e = g[v][i];
            const Edge<T> toward_v{e.to, v, e.cost, e.idx};
            incoming[i] = e.idx == tree.parent_edge[v] ? from_parent[v] : lift(down[e.to], toward_v);
            prefix[i + 1] = merge(prefix[i], incoming[i]);
        }
        answer[v] = add_vertex(prefix[degree], v);
        S suffix = identity;
        for (int i = degree - 1; i >= 0; --i) {
            const auto& e = g[v][i];
            if (e.idx != tree.parent_edge[v]) {
                const S excluded = merge(prefix[i], suffix);
                from_parent[e.to] = lift(add_vertex(excluded, v), e);
            }
            suffix = merge(incoming[i], suffix);
        }
    }
    return answer;
}

} // namespace cp
