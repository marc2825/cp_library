#pragma once
// 説明: docs/graph_flow/stoer_wagner.md
#include <algorithm>
#include <numeric>
#include <optional>
#include <stdexcept>
#include <type_traits>
#include <vector>
#include "../graph/graph.hpp"
#include "../combinatorial_optimization/detail/arithmetic.hpp"

namespace cp {
template<class Cap> struct GlobalMinCutResult { Cap cost; std::vector<bool> side; };
template<class Cap> std::optional<GlobalMinCutResult<Cap>> stoer_wagner(const Graph<Cap>& g) {
    static_assert(std::is_integral<Cap>::value && sizeof(Cap)<=8,"integer capacity required");
    if(g.directed()) throw std::invalid_argument("global cut requires undirected graph");
    int n=g.size(); if(n<2) return std::nullopt; using W=OptimizationWide;
    std::vector<std::vector<W>> adjacency(n,std::vector<W>(n));
    for(const auto& e:g.edges()) {
        if(e.cost<Cap(0)) throw std::invalid_argument("negative cut capacity");
        if(e.from!=e.to) { adjacency[e.from][e.to]=optimization_detail::add(adjacency[e.from][e.to],W(e.cost)); adjacency[e.to][e.from]=adjacency[e.from][e.to]; }
    }
    std::vector<int> active(n); std::iota(active.begin(),active.end(),0);
    std::vector<std::vector<int>> groups(n); for(int v=0;v<n;++v) groups[v]={v};
    std::optional<W> best; std::vector<bool> side(n);
    while(active.size()>1) {
        std::vector<W> weight(n); std::vector<bool> used(n); int previous=-1;
        for(std::size_t step=0;step<active.size();++step) {
            int v=-1; for(int u:active) if(!used[u] && (v<0 || weight[v]<weight[u])) v=u;
            if(step+1==active.size()) {
                if(!best || weight[v]<*best) { best=weight[v]; std::fill(side.begin(),side.end(),false); for(int u:groups[v]) side[u]=true; }
                for(int u:active) if(u!=previous && u!=v) { adjacency[previous][u]=optimization_detail::add(adjacency[previous][u],adjacency[v][u]); adjacency[u][previous]=adjacency[previous][u]; }
                groups[previous].insert(groups[previous].end(),groups[v].begin(),groups[v].end()); active.erase(std::find(active.begin(),active.end(),v)); break;
            }
            used[v]=true; previous=v;
            for(int u:active) if(!used[u]) weight[u]=optimization_detail::add(weight[u],adjacency[v][u]);
        }
    }
    return GlobalMinCutResult<Cap>{optimization_detail::narrow<Cap>(*best),std::move(side)};
}
} // namespace cp
