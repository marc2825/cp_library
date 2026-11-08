#pragma once

#include <type_traits>
#include <vector>
#include "rooted_tree.hpp"

namespace cp {
namespace detail {

template <class S, class T, class Merge, class AddVertex, class Lift>
auto tree_dp_bottom_up(const Graph<T>& g, const RootedTree<T>& tree, const S& identity,
                       Merge& merge, AddVertex& add_vertex, Lift& lift) {
    using R = std::decay_t<std::invoke_result_t<AddVertex&, const S&, int>>;
    std::vector<R> dp(g.size());
    for (int v : tree.postorder) {
        S aggregate = identity;
        for (const auto& e : g[v]) {
            if (e.idx == tree.parent_edge[v]) continue;
            const Edge<T> toward_parent{e.to, v, e.cost, e.idx};
            aggregate = merge(aggregate, lift(dp[e.to], toward_parent));
        }
        dp[v] = add_vertex(aggregate, v);
    }
    return dp;
}

} // namespace detail

// 子の寄与 S を隣接順で併合し、頂点を加えて R にする非再帰 Tree DP。
// merge(S,S)->S, add_vertex(S,v)->R, lift(R, Edge{送信元,受信先,...})->S。
// R はデフォルト構築可能。詳細: docs/tree/tree_dp.md。
template <class S, class T, class Merge, class AddVertex, class Lift>
auto tree_dp(const Graph<T>& g, const S& identity, Merge merge,
             AddVertex add_vertex, Lift lift, int root = 0) {
    RootedTree<T> tree(g, root);
    return detail::tree_dp_bottom_up(g, tree, identity, merge, add_vertex, lift);
}

} // namespace cp
