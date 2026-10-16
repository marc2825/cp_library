#pragma once
#include <limits>
#include <stdexcept>
#include <vector>
#include "dinic.hpp"

namespace cp {

template <class Cap>
struct MinimumCutResult {
    Cap capacity{};
    std::vector<bool> source_side;
    std::vector<int> edge_ids;
};

// Completes the flow in place, then restores the original S->T edges (including zero capacity).
template <class Cap>
MinimumCutResult<Cap> minimum_cut(Dinic<Cap>& network, int s, int t) {
    network.max_flow(s, t);
    MinimumCutResult<Cap> result;
    result.source_side = network.min_cut(s);
    if (result.source_side[t]) throw std::overflow_error("maximum flow exceeds capacity type");
    for (int id = 0; id < network.edge_count(); ++id) {
        const auto e = network.get_edge(id);
        if (result.source_side[e.from] && !result.source_side[e.to]) {
            if (e.capacity > std::numeric_limits<Cap>::max() - result.capacity)
                throw std::overflow_error("minimum cut capacity exceeds capacity type");
            result.capacity += e.capacity;
            result.edge_ids.push_back(id);
        }
    }
    return result;
}

} // namespace cp
