#pragma once

#include <algorithm>
#include <cassert>
#include <stdexcept>
#include <utility>
#include <vector>
#include "rooted_tree.hpp"

namespace cp {

template <class T>
struct TreeDiameterResult {
    T length = T(0);
    std::pair<int, int> endpoints{-1, -1};
    std::vector<int> vertices, edges;
};

// 非負重みの連結な木限定。O(N)。空の木は長さ0・端点(-1,-1)。
template <class T>
TreeDiameterResult<T> tree_diameter(const Graph<T>& g) {
    TreeDiameterResult<T> result;
    if (g.size() == 0) return result;
    for (const auto& e : g.edges()) { assert(e.cost >= T(0)); (void)e; }
    RootedTree<T> first(g);
    if (first.roots.size() != 1) throw std::invalid_argument("diameter requires a connected tree");
    int s = static_cast<int>(std::max_element(first.distance_from_root.begin(), first.distance_from_root.end())
        - first.distance_from_root.begin());
    RootedTree<T> second(g, s);
    int t = static_cast<int>(std::max_element(second.distance_from_root.begin(), second.distance_from_root.end())
        - second.distance_from_root.begin());
    result.length = second.distance_from_root[t]; result.endpoints = {s, t};
    for (int v = t; v != -1; v = second.parent[v]) {
        result.vertices.push_back(v);
        if (second.parent_edge[v] != -1) result.edges.push_back(second.parent_edge[v]);
    }
    std::reverse(result.vertices.begin(), result.vertices.end());
    std::reverse(result.edges.begin(), result.edges.end());
    return result;
}

} // namespace cp
