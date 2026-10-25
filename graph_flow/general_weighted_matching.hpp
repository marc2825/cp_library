#pragma once
// 説明: docs/graph_flow/general_weighted_matching.md
#include <boost/graph/adjacency_list.hpp>
#include "detail/weighted_blossom.hpp"
#include <algorithm>
#include <map>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <vector>
#include "../graph/graph.hpp"
#include "../combinatorial_optimization/detail/arithmetic.hpp"
namespace cp {
struct GeneralWeightedMatchingResult {
    OptimizationWide weight=0;
    std::vector<int> mate,mate_edge,edge_ids;
};
// 空 matching を許す最大重み。cardinality は最大化しない。Boost Graph が必要。
template<class T> GeneralWeightedMatchingResult general_weighted_matching(const Graph<T>& g) {
    static_assert(std::is_integral_v<T> && sizeof(T)<=8 && !std::is_same_v<T,bool>);
    if(g.directed()) throw std::invalid_argument("undirected matching required");
    using W=OptimizationWide;
    using B=boost::adjacency_list<boost::vecS,boost::vecS,boost::undirectedS,boost::no_property,boost::property<boost::edge_weight_t,W>>;
    B b(g.size()); std::map<std::pair<int,int>,int> best;
    for(auto e:g.edges()) if(e.from!=e.to && e.cost>T(0)) {
        auto key=std::minmax(e.from,e.to); auto it=best.find(key);
        if(it==best.end() || g.edges()[it->second].cost<e.cost) best[{key.first,key.second}]=e.idx;
    }
    for(auto entry:best) { auto e=g.edges()[entry.second]; boost::add_edge(e.from,e.to,W(e.cost),b); }
    GeneralWeightedMatchingResult r; r.mate.assign(g.size(),-1); r.mate_edge=r.mate;
    if(best.empty()) return r;
    using Vertex=typename boost::graph_traits<B>::vertex_descriptor;
    std::vector<Vertex> mate(g.size()); auto map=boost::make_iterator_property_map(mate.begin(),get(boost::vertex_index,b));
    auto indices=get(boost::vertex_index,b); auto weights=get(boost::edge_weight,b);
    weighted_blossom_detail::maximum_weighted_matching_context<B,decltype(indices),decltype(weights)> matching(b,indices,weights);
    matching.run(); matching.extract_matching(map);
    for(int v=0;v<g.size();++v) if(mate[v]!=boost::graph_traits<B>::null_vertex()) {
        int u=int(mate[v]),id=best.at(std::minmax(v,u)); r.mate[v]=u; r.mate_edge[v]=id;
        if(v<u) { r.edge_ids.push_back(id); r.weight=optimization_detail::add(r.weight,W(g.edges()[id].cost)); }
    }
    return r;
}
} // namespace cp
