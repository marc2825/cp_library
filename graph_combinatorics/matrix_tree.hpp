#pragma once
#include <cassert>
#include <cstdint>
#include <type_traits>
#include <utility>
#include <vector>
#include "../graph/graph.hpp"
#include "determinant_mod.hpp"

namespace cp {
namespace graph_combinatorics_detail {
template<int Mod, class T, class Weight>
int matrix_tree(const Graph<T>& g, int removed, Weight weight) {
    static_assert(is_prime_mod(Mod), "Mod must be a prime int");
    assert(!g.directed());
    if (g.size() == 0) { assert(removed == -1); return 0; }
    if (removed == -1) removed = g.size() - 1;
    assert(0 <= removed && removed < g.size());
    const int n = g.size() - 1;
    std::vector<std::vector<long long>> minor(n, std::vector<long long>(n));
    for (const auto& e : g.edges()) {
        if (e.from == e.to) continue;
        const long long w = weight(e.cost);
        const int u = e.from - (e.from > removed), v = e.to - (e.to > removed);
        if (e.from != removed) minor[u][u] = (minor[u][u] + w) % Mod;
        if (e.to != removed) minor[v][v] = (minor[v][v] + w) % Mod;
        if (e.from != removed && e.to != removed) {
            minor[u][v] = (minor[u][v] + Mod - w) % Mod;
            minor[v][u] = (minor[v][u] + Mod - w) % Mod;
        }
    }
    return determinant_mod<Mod>(std::move(minor));
}
} // namespace graph_combinatorics_detail

// Undirected multigraph; loops ignored, empty graph=0, singleton=1.
template<int Mod = 998244353, class T>
int count_spanning_trees(const Graph<T>& g, int removed_vertex = -1) {
    return graph_combinatorics_detail::matrix_tree<Mod>(g, removed_vertex, [](const T&) { return 1LL; });
}
// Sum of products of integer edge weights over all spanning trees, modulo prime Mod.
template<int Mod = 998244353, class T>
int count_weighted_spanning_trees(const Graph<T>& g, int removed_vertex = -1) {
    static_assert(std::is_integral<T>::value && !std::is_same<T, bool>::value, "integer edge weights required");
    return graph_combinatorics_detail::matrix_tree<Mod>(g, removed_vertex, [](T value) {
        if constexpr (std::is_signed<T>::value) {
            long long residue = static_cast<long long>(value) % Mod;
            return residue < 0 ? residue + Mod : residue;
        } else return static_cast<long long>(static_cast<std::uint64_t>(value) % Mod);
    });
}
} // namespace cp
