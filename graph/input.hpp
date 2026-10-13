#pragma once

#include <cassert>
#include <istream>
#include <stdexcept>
#include "graph.hpp"

namespace cp {

struct GraphInputOptions {
    GraphKind kind = GraphKind::Undirected;
    bool weighted = false;
    int index_base = 1;
};

template <class T = long long>
Graph<T> read_graph(std::istream& input, int n, int m, GraphInputOptions options = {}) {
    assert(m >= 0);
    Graph<T> g(n, options.kind);
    for (int i = 0; i < m; ++i) {
        int from, to;
        T cost = T(1);
        if (!(input >> from >> to) || (options.weighted && !(input >> cost)))
            throw std::runtime_error("incomplete graph input");
        g.add_edge(from - options.index_base, to - options.index_base, cost);
    }
    return g;
}

} // namespace cp
