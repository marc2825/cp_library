#pragma once
#include <algorithm>
#include <optional>
#include <stdexcept>
#include <type_traits>
#include <vector>
#include "graph.hpp"
#include "../combinatorial_optimization/detail/arithmetic.hpp"

namespace cp {
struct NegativeCycle {
    std::vector<int> vertices, edge_ids; // Closed vertices: size = edge_ids.size()+1.
    OptimizationWide weight=0;
};
namespace detail {
struct BellmanFordWitness {
    std::vector<std::optional<OptimizationWide>> distance;
    std::optional<NegativeCycle> cycle;
};
template<class T> BellmanFordWitness bellman_ford_witness(const Graph<T>& g, int source) {
    static_assert(std::is_integral_v<T> && sizeof(T)<=8 && !std::is_same_v<T,bool>);
    int n=g.size();
    if (source < -1 || source>=n) throw std::out_of_range("negative cycle source");
    BellmanFordWitness r; r.distance.resize(n);
    if (source<0) for (auto& d:r.distance) d=0;
    else r.distance[source]=0;
    std::vector<int> parent(n,-1), edge(n,-1); int changed=-1;
    for (int round=0; round<n; ++round) {
        changed=-1;
        for (int u=0; u<n; ++u) if (r.distance[u]) for (auto e:g[u]) {
            auto value=optimization_detail::add(*r.distance[u],OptimizationWide(e.cost));
            if (!r.distance[e.to] || value<*r.distance[e.to]) {
                r.distance[e.to]=value; parent[e.to]=u; edge[e.to]=e.idx; changed=e.to;
            }
        }
        if (changed<0) return r;
    }
    if (changed<0) return r;
    for (int i=0; i<n; ++i) changed=parent[changed];
    NegativeCycle cycle; int v=changed;
    do {
        cycle.vertices.push_back(v); cycle.edge_ids.push_back(edge[v]);
        cycle.weight=optimization_detail::add(cycle.weight,OptimizationWide(g.edges()[edge[v]].cost));
        v=parent[v];
    } while (v!=changed);
    cycle.vertices.push_back(changed);
    std::reverse(cycle.vertices.begin(),cycle.vertices.end());
    std::reverse(cycle.edge_ids.begin(),cycle.edge_ids.end());
    r.cycle=std::move(cycle); return r;
}
}
// source=-1 searches every component. An undirected negative edge can be used twice.
template<class T> std::optional<NegativeCycle> find_negative_cycle(const Graph<T>& g, int source=-1) {
    return detail::bellman_ford_witness(g,source).cycle;
}
} // namespace cp
