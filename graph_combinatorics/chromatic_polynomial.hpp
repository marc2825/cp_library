#pragma once
// 説明: docs/graph_combinatorics/chromatic_polynomial.md
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <utility>
#include <vector>
#include "../graph/graph.hpp"
namespace cp {
// 独立集合による集合分割DP。係数は昇冪順。Ringは整数から構築可能な環。
template<class Ring, class T> std::vector<Ring> chromatic_polynomial(const Graph<T>& g) {
    if(g.directed()) throw std::invalid_argument("chromatic polynomial requires undirected graph");
    int n=g.size(); if(n>20) throw std::length_error("chromatic polynomial requires n<=20");
    std::vector<std::uint32_t> adjacency(n);
    for(auto e:g.edges()) { if(e.from==e.to) return std::vector<Ring>(n+1,Ring(0)); adjacency[e.from]|=std::uint32_t(1)<<e.to; adjacency[e.to]|=std::uint32_t(1)<<e.from; }
    std::size_t states=std::size_t(1)<<n;
    std::vector<bool> independent(states,true);
    for(std::size_t s=1;s<states;++s) { int v=__builtin_ctz(static_cast<unsigned>(s)); auto rest=s&(s-1); independent[s]=independent[rest] && !(adjacency[v]&rest); }
    std::vector<Ring> dp(states*(n+1),Ring(0)); dp[0]=Ring(1);
    for(std::size_t s=1;s<states;++s) {
        auto first=s&(~s+1), rest=s^first;
        for(auto sub=rest;;sub=(sub-1)&rest) {
            auto block=sub|first;
            if(independent[block]) for(int k=1;k<=n;++k) dp[s*(n+1)+k]=dp[s*(n+1)+k]+dp[(s^block)*(n+1)+k-1];
            if(sub==0) break;
        }
    }
    std::vector<Ring> result(n+1,Ring(0)),falling{Ring(1)};
    for(int k=0;k<=n;++k) {
        for(int j=0;j<=k;++j) result[j]=result[j]+dp[(states-1)*(n+1)+k]*falling[j];
        if(k==n) break;
        std::vector<Ring> next(k+2,Ring(0));
        for(int j=0;j<=k;++j) { next[j]=next[j]-falling[j]*Ring(k); next[j+1]=next[j+1]+falling[j]; } falling=std::move(next);
    }
    return result;
}
} // namespace cp
