#pragma once
#include <algorithm>
#include <array>
#include <utility>
#include <vector>
#include "detail/bit_graph.hpp"

namespace cp {
// Simple undirected graph; emit(u,v,w) returns false to stop. u<v<w.
template<class T, class Emit>
bool enumerate_triangles(const Graph<T>& g, Emit emit) {
    graph_combinatorics_detail::assert_simple_undirected(g);
    std::vector<std::vector<int>> forward(g.size());
    for (const auto& e : g.edges()) {
        int u = e.from, v = e.to;
        if (std::make_pair(g[u].size(), u) > std::make_pair(g[v].size(), v)) std::swap(u, v);
        forward[u].push_back(v);
    }
    std::vector<int> mark(g.size(), -1);
    for (int u = 0; u < g.size(); ++u) {
        for (int v : forward[u]) mark[v] = u;
        for (int v : forward[u]) for (int w : forward[v]) if (mark[w] == u) {
            std::array<int, 3> triangle{u, v, w};
            std::sort(triangle.begin(), triangle.end());
            if (!emit(triangle[0], triangle[1], triangle[2])) return false;
        }
    }
    return true;
}
} // namespace cp
