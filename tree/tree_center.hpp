#pragma once
// 木の中心（重心とは異なる）。説明: docs/tree/tree_center.md
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <type_traits>
#include <vector>
#include "diameter.hpp"

namespace cp {
struct TreeCenterResult {
    int radius = 0;
    std::vector<int> vertices;
};

// 辺の重みは無視する。空なら radius=0, vertices={}。O(n)。
template<class T> TreeCenterResult tree_center(const Graph<T>& g) {
    Graph<int> unit(g.size(), g.kind());
    for (auto e : g.edges()) unit.add_edge(e.from, e.to);
    auto d = tree_diameter(unit);
    TreeCenterResult result;
    if (d.vertices.empty()) return result;
    result.radius = (d.length + 1) / 2;
    result.vertices.push_back(d.vertices[d.length / 2]);
    if (d.length % 2) result.vertices.push_back(d.vertices[d.length / 2 + 1]);
    std::sort(result.vertices.begin(), result.vertices.end());
    return result;
}

template<class T> struct WeightedTreeCenterResult {
    std::vector<int> vertices; // 離心度が最小の頂点（昇順）
    T vertex_radius = T(0), radius_twice = T(0);
    // 連続中心の代表。頂点なら vertex>=0。辺内部なら edge_id>=0。
    int vertex = -1, edge_id = -1, from = -1, to = -1;
    T offset_twice = T(0); // from からの距離の2倍。整数でも半整数を厳密に表現。
};

// 非負の有限重み。距離はTの有限範囲内（max未満）。O(n)。
template<class T> WeightedTreeCenterResult<T> weighted_tree_center(const Graph<T>& g) {
    for (auto e : g.edges()) {
        if (!(e.cost >= T(0))) throw std::invalid_argument("negative/NaN tree weight");
        if constexpr (std::is_floating_point_v<T>)
            if (!std::isfinite(e.cost)) throw std::invalid_argument("non-finite tree weight");
    }
    auto d = tree_diameter(g);
    WeightedTreeCenterResult<T> result;
    if (g.size() == 0) return result;
    RootedTree<T> a(g, d.endpoints.first), b(g, d.endpoints.second);
    result.vertex_radius = d.length;
    result.radius_twice = d.length;
    for (int v = 0; v < g.size(); ++v) {
        T eccentricity = std::max(a.distance_from_root[v], b.distance_from_root[v]);
        if (eccentricity < result.vertex_radius) {
            result.vertex_radius = eccentricity;
            result.vertices.clear();
        }
        if (eccentricity == result.vertex_radius) result.vertices.push_back(v);
    }
    for (std::size_t i = 0; i < d.vertices.size(); ++i) {
        int v = d.vertices[i];
        T prefix = a.distance_from_root[v], suffix = d.length - prefix;
        if (prefix == suffix) { result.vertex = v; break; }
        if (prefix > suffix) {
            result.edge_id = d.edges[i - 1];
            result.from = d.vertices[i - 1]; result.to = v;
            T before = a.distance_from_root[result.from];
            result.offset_twice = (d.length - before) - before;
            break;
        }
    }
    return result;
}
} // namespace cp
