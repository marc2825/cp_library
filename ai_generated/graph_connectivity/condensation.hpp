#pragma once
#include <vector>
#include "scc.hpp"

// SCC DAG. Internal edges are omitted. By default parallel component arcs are merged.
// original_edge[dag edge ID] is one original edge ID, not necessarily the smallest.
// Docs: docs/graph_connectivity/condensation.md
namespace cp {

struct SCCCondensationResult {
    StronglyConnectedComponentsResult scc;
    Graph<int> dag{0, GraphKind::Directed};
    std::vector<int> original_edge;
};

template <class T>
SCCCondensationResult condensation_graph(const Graph<T>& g, bool deduplicate = true) {
    SCCCondensationResult result;
    result.scc = strongly_connected_components(g);
    result.dag = Graph<int>(result.scc.size(), GraphKind::Directed);
    std::vector<int> seen(result.scc.size(), -1);
    for (int from = 0; from < result.scc.size(); ++from) {
        for (int v : result.scc.groups[from]) for (const auto& e : g[v]) {
            int to = result.scc.component[e.to];
            if (from == to || (deduplicate && seen[to] == from)) continue;
            seen[to] = from;
            result.dag.add_edge(from, to, 1);
            result.original_edge.push_back(e.idx);
        }
    }
    return result;
}

} // namespace cp
