#pragma once
#include <stdexcept>
#include <type_traits>
#include <vector>
#include "../graph/graph.hpp"
#include "../combinatorial_optimization/detail/arithmetic.hpp"
#include "dinic.hpp"

namespace cp {
// n-1 max flows; minimum edge on a tree path equals the pair minimum cut.
// Nonnegative integral capacities, with their total representable by Cap.
template<class Cap> Graph<Cap> gomory_hu_tree(const Graph<Cap>& g) {
    static_assert(std::is_integral_v<Cap> && sizeof(Cap)<=8 && !std::is_same_v<Cap,bool>);
    if (g.directed()) throw std::invalid_argument("Gomory-Hu requires undirected graph");
    int n=g.size(); Dinic<Cap> flow(n); OptimizationWide total=0;
    for (const auto& e:g.edges()) {
        if (e.cost<0) throw std::invalid_argument("negative capacity");
        if (e.from==e.to) continue;
        total=optimization_detail::add(total,OptimizationWide(e.cost));
        flow.add_edge(e.from,e.to,e.cost); flow.add_edge(e.to,e.from,e.cost);
    }
    (void)optimization_detail::narrow<Cap>(total);
    std::vector<int> parent(n,0); std::vector<Cap> value(n);
    for (int s=1; s<n; ++s) {
        int t=parent[s]; flow.reset_flow(); Cap f=flow.max_flow(s,t);
        auto side=flow.min_cut(s);
        for (int v=s+1; v<n; ++v) if (parent[v]==t && side[v]) parent[v]=s;
        if (side[parent[t]]) {
            parent[s]=parent[t]; parent[t]=s; value[s]=value[t]; value[t]=f;
        } else value[s]=f;
    }
    Graph<Cap> tree(n);
    for (int v=1; v<n; ++v) tree.add_edge(v,parent[v],value[v]);
    return tree;
}
} // namespace cp
