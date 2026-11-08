#pragma once
#include <cassert>
#include <cstdint>
#include <stdexcept>
#include <vector>
#include "../../graph/graph.hpp"

namespace cp { namespace graph_combinatorics_detail {
using Mask = std::uint64_t;
inline Mask all_vertices(int n) { return n == 64 ? ~Mask{} : (Mask{1} << n) - 1; }
inline int first_vertex(Mask bits) {
    assert(bits != 0);
    int result = 0;
    if (!(bits & 0xffffffffULL)) { bits >>= 32; result += 32; }
    if (!(bits & 0xffffULL)) { bits >>= 16; result += 16; }
    if (!(bits & 0xffULL)) { bits >>= 8; result += 8; }
    if (!(bits & 0xfULL)) { bits >>= 4; result += 4; }
    if (!(bits & 0x3ULL)) { bits >>= 2; result += 2; }
    if (!(bits & 0x1ULL)) ++result;
    return result;
}
inline int bit_count(Mask x) {
    x -= (x >> 1) & 0x5555555555555555ULL;
    x = (x & 0x3333333333333333ULL) + ((x >> 2) & 0x3333333333333333ULL);
    x = (x + (x >> 4)) & 0x0f0f0f0f0f0f0f0fULL;
    return static_cast<int>((x * 0x0101010101010101ULL) >> 56);
}
template<class T> void assert_simple_undirected(const Graph<T>& g) {
    assert(!g.directed());
#ifndef NDEBUG
    std::vector<int> seen(g.size(), -1);
    for (int v = 0; v < g.size(); ++v) for (const auto& e : g[v]) {
        assert(e.to != v && seen[e.to] != v);
        seen[e.to] = v;
    }
#else
    (void)g;
#endif
}
template<class T> std::vector<Mask> adjacency_masks(const Graph<T>& g) {
    if (g.size() > 64) throw std::length_error("bitset graph supports at most 64 vertices");
    assert_simple_undirected(g);
    std::vector<Mask> adjacency(g.size());
    for (const auto& e : g.edges()) {
        adjacency[e.from] |= Mask{1} << e.to;
        adjacency[e.to] |= Mask{1} << e.from;
    }
    return adjacency;
}
} } // namespace cp::graph_combinatorics_detail
