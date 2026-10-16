#pragma once
#include <vector>
#include "maximum_clique.hpp"

namespace cp {
// Exact maximum independent set = a maximum clique in the loop-free complement.
template<class T> std::vector<int> maximum_independent_set(const Graph<T>& g) {
    using namespace graph_combinatorics_detail;
    auto adjacency = adjacency_masks(g);
    const Mask all = all_vertices(g.size());
    for (int v = 0; v < g.size(); ++v) adjacency[v] = all & ~adjacency[v] & ~(Mask{1} << v);
    return maximum_clique_masks(adjacency);
}
} // namespace cp
