#pragma once
#include <vector>
#include "graph.hpp"

// Docs: docs/graph_basic/spanning_forest.md
namespace cp {
template <class T>
struct MinimumSpanningForestResult {
    T cost = T(0);
    std::vector<Edge<T>> edges;
    int component_count = 0;
    bool connected() const { return component_count <= 1; }
};

} // namespace cp
