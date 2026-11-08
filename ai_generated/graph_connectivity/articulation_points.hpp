#pragma once
#include <vector>
#include "low_link.hpp"

// Articulation vertices, sorted in ascending order. O(V+E).
// Docs: docs/graph_connectivity/articulation_points.md
namespace cp {
template <class T> std::vector<int> articulation_points(const Graph<T>& g) {
    return low_link(g).articulation;
}
} // namespace cp
