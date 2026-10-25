#pragma once
#include <algorithm>
#include <optional>
#include <limits>
#include <queue>
#include <stdexcept>
#include <type_traits>
#include <vector>
#include "disjoint_paths.hpp"
#include "../combinatorial_optimization/min_cost_flow.hpp"

namespace cp {
template<class Cost> struct MinCostDisjointPathsResult : DisjointPathsResult {Cost cost=0;};
// Minimum total cost for exactly k paths. Nonnegative signed integer costs.
// Internal vertices are exclusive in vertex mode; original edges are exclusive in both.
template<class Cost> std::optional<MinCostDisjointPathsResult<Cost>> min_cost_disjoint_paths(
        const Graph<Cost>& g,int source,int sink,int k,bool vertex_disjoint=false) {
    static_assert(std::is_integral_v<Cost> && std::is_signed_v<Cost> && sizeof(Cost)<=8);
    int n=g.size();
    if (source<0 || sink<0 || source>=n || sink>=n || source==sink || k<0)
        throw std::invalid_argument("invalid disjoint path terminals/count");
    if (n>std::numeric_limits<int>::max()/2) throw std::length_error("split network size");
    for (auto e:g.edges()) if (e.cost<0) throw std::invalid_argument("nonnegative path costs required");
    MinCostDisjointPathsResult<Cost> r; if (!k) return r;
    MinCostFlow<int,Cost> flow(2*n);
    for (int v=0;v<n;++v) flow.add_edge(2*v,2*v+1,vertex_disjoint && v!=source && v!=sink?1:k,0);
    struct Arc {int from,to,id,network;}; std::vector<Arc> arcs;
    for (auto e:g.edges()) if (e.from!=e.to) {
        arcs.push_back({e.from,e.to,e.idx,flow.add_edge(2*e.from+1,2*e.to,1,e.cost)});
        if (!g.directed()) arcs.push_back({e.to,e.from,e.idx,flow.add_edge(2*e.to+1,2*e.from,1,e.cost)});
    }
    auto answer=flow.flow(2*source+1,2*sink,k); if (answer.flow<k) return std::nullopt;
    r.cost=answer.cost;std::vector<int> remaining(arcs.size());
    for (int i=0;i<int(arcs.size());++i) remaining[i]=flow.get_edge(arcs[i].network).flow;
    // Opposite undirected uses form a circulation. Positive-cost ones cannot be optimal.
    if (!g.directed()) for (int i=0;i<int(arcs.size());i+=2) {
        int cancel=std::min(remaining[i],remaining[i+1]);remaining[i]-=cancel;remaining[i+1]-=cancel;
    }
    std::vector<std::vector<int>> adj(n);
    for (int i=0;i<int(arcs.size());++i) adj[arcs[i].from].push_back(i);
    for (int iteration=0;iteration<k;++iteration) {
        std::vector<int> parent(n,-1),queue{source};parent[source]=-2;
        for (std::size_t head=0;head<queue.size() && parent[sink]<0;++head)
            for (int id:adj[queue[head]]) if (remaining[id] && parent[arcs[id].to]==-1) {
                parent[arcs[id].to]=id;queue.push_back(arcs[id].to);
            }
        if (parent[sink]<0) throw std::logic_error("minimum cost flow decomposition");
        std::vector<int> path{sink},ids;
        for (int v=sink;v!=source;) {
            int id=parent[v];--remaining[id];ids.push_back(arcs[id].id);v=arcs[id].from;path.push_back(v);
        }
        std::reverse(path.begin(),path.end());std::reverse(ids.begin(),ids.end());
        r.paths.push_back(std::move(path));r.edge_ids.push_back(std::move(ids));
    }
    return r;
}
template<class Cost> auto min_cost_edge_disjoint_paths(const Graph<Cost>& g,int s,int t,int k) {
    return min_cost_disjoint_paths(g,s,t,k,false);
}
template<class Cost> auto min_cost_vertex_disjoint_paths(const Graph<Cost>& g,int s,int t,int k) {
    return min_cost_disjoint_paths(g,s,t,k,true);
}
} // namespace cp
