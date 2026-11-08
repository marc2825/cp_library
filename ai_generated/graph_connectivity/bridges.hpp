#pragma once
#include <vector>
#include "low_link.hpp"

// Logical edge IDs, sorted in ascending order. O(V+E). Use low_link for both outputs.
// Docs: docs/graph_connectivity/bridges.md
namespace cp {
template <class T> std::vector<int> bridges(const Graph<T>& g) {
    return low_link(g).bridge_edges;
}
} // namespace cp
