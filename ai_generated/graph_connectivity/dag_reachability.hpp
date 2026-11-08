#pragma once
#include <algorithm>
// 説明: docs/graph_connectivity/dag_reachability.md
#include <cassert>
#include <cstdint>
#include <queue>
#include <stdexcept>
#include <utility>
#include <vector>
#include "graph.hpp"

namespace cp {
namespace detail {
template<class T> std::vector<int> reachability_topological_order(const Graph<T>& g) {
    if(!g.directed()) throw std::invalid_argument("DAG reachability requires directed graph");
    std::vector<int> degree(g.size()),order; for(const auto& e:g.edges()) ++degree[e.to];
    std::queue<int> queue; for(int v=0;v<g.size();++v) if(!degree[v]) queue.push(v);
    while(!queue.empty()) { int v=queue.front(); queue.pop(); order.push_back(v); for(const auto& e:g[v]) if(--degree[e.to]==0) queue.push(e.to); }
    if(int(order.size())!=g.size()) throw std::invalid_argument("graph is not a DAG");
    return order;
}
}
// 全点bit行列。自身への長さ0の到達もtrue。
class DAGReachability {
    int n_; std::vector<std::vector<std::uint64_t>> reach_;
public:
    template<class T> explicit DAGReachability(const Graph<T>& g) : n_(g.size()),reach_(n_,std::vector<std::uint64_t>((n_+63LL)/64)) {
        auto order=detail::reachability_topological_order(g);
        for(auto it=order.rbegin();it!=order.rend();++it) {
            int v=*it; reach_[v][v/64]|=std::uint64_t(1)<<(v%64);
            for(const auto& e:g[v]) for(std::size_t k=0;k<reach_[v].size();++k) reach_[v][k]|=reach_[e.to][k];
        }
    }
    bool reachable(int u,int v) const { assert(0<=u && u<n_ && 0<=v && v<n_); return (reach_[u][v/64]>>(v%64))&1; }
};
// 大きいDAGは64問ずつの伝播で、V^2の領域を避ける。
template<class T> std::vector<bool> dag_reachability(const Graph<T>& g,const std::vector<std::pair<int,int>>& queries) {
    auto order=detail::reachability_topological_order(g); std::vector<bool> answer(queries.size());
    for(auto [u,v]:queries) { assert(0<=u && u<g.size() && 0<=v && v<g.size()); (void)u; (void)v; }
    for(std::size_t start=0;start<queries.size();start+=64) {
        std::vector<std::uint64_t> reach(g.size()); auto count=std::min(std::size_t(64),queries.size()-start);
        for(std::size_t i=0;i<count;++i) reach[queries[start+i].first]|=std::uint64_t(1)<<i;
        for(int v:order) for(const auto& e:g[v]) reach[e.to]|=reach[v];
        for(std::size_t i=0;i<count;++i) answer[start+i]=(reach[queries[start+i].second]>>i)&1;
    }
    return answer;
}
} // namespace cp
