#pragma once
// 説明: docs/graph_combinatorics/count_four_cycles.md
#include <algorithm>
#include <cstdint>
#include <stdexcept>
#include <utility>
#include <vector>
#include "../graph/graph.hpp"
#include "../combinatorial_optimization/detail/arithmetic.hpp"
namespace cp {
// 単純無向グラフ。弦があっても長さ4の閉路として数える。
template<class T> std::uint64_t count_four_cycles(const Graph<T>& g) {
    if(g.directed()) throw std::invalid_argument("C4 count requires simple undirected graph");
    int n=g.size(); std::vector<std::vector<int>> adj(n);
    for(auto e:g.edges()) { if(e.from==e.to) throw std::invalid_argument("C4 self loop"); adj[e.from].push_back(e.to); adj[e.to].push_back(e.from); }
    for(auto& row:adj) { std::sort(row.begin(),row.end()); if(std::adjacent_find(row.begin(),row.end())!=row.end()) throw std::invalid_argument("C4 parallel edge"); }
    auto below=[&](int u,int v){return std::make_pair(adj[u].size(),u)<std::make_pair(adj[v].size(),v);};
    std::vector<int> count(n),touched; OptimizationWide answer=0;
    for(int v=0;v<n;++v) {
        for(int u:adj[v]) if(below(u,v)) for(int w:adj[u]) if(below(w,v)) {
            answer=optimization_detail::add(answer,count[w]); if(count[w]++==0) touched.push_back(w);
        }
        for(int w:touched) count[w]=0;
        touched.clear();
    }
    return optimization_detail::narrow<std::uint64_t>(answer);
}
} // namespace cp
