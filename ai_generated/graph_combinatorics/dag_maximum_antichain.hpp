#pragma once
#include <vector>
#include "../graph_connectivity/dag_reachability.hpp"
#include "../graph_flow/bipartite_vertex_cover.hpp"

namespace cp {
struct DAGAntichainResult {
    std::vector<int> antichain;
    std::vector<std::vector<int>> chains; // Consecutive elements are reachable, not necessarily adjacent.
};
// Dilworth on the strict transitive closure; directed DAG required.
template<class T> DAGAntichainResult dag_maximum_antichain(const Graph<T>& g) {
    DAGReachability reach(g); int n=g.size();
    std::vector<std::pair<int,int>> edges;
    for (int u=0; u<n; ++u) for (int v=0; v<n; ++v)
        if (u!=v && reach.reachable(u,v)) edges.emplace_back(u,v);
    auto cover=bipartite_vertex_cover(n,n,edges);
    DAGAntichainResult r;
    for (int v=0; v<n; ++v) {
        if (cover.reachable_left[v] && !cover.reachable_right[v]) r.antichain.push_back(v);
        if (cover.matching.right_match[v]>=0) continue;
        std::vector<int> chain;
        for (int u=v; u>=0; u=cover.matching.left_match[u]) chain.push_back(u);
        r.chains.push_back(std::move(chain));
    }
    return r;
}
} // namespace cp
