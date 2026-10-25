#pragma once
#include <numeric>
#include <optional>
#include <stdexcept>
#include <type_traits>
#include <vector>
#include "cycle.hpp"
#include "../combinatorial_optimization/detail/arithmetic.hpp"

namespace cp {
struct MinimumMeanCycle {
    OptimizationWide numerator=0;
    int denominator=1;
    std::vector<int> vertices,edge_ids; // Closed vertex sequence.
};
// Karp DP with exact rational comparison; directed integer graph. O(VE+V^2), O(V^2+E).
template<class T> std::optional<MinimumMeanCycle> minimum_mean_cycle(const Graph<T>& g) {
    static_assert(std::is_integral_v<T> && sizeof(T)<=8 && !std::is_same_v<T,bool>);
    if (!g.directed()) throw std::invalid_argument("minimum mean cycle requires directed graph");
    int n=g.size(); if (!n) return std::nullopt;
    using W=OptimizationWide; using O=std::optional<W>;
    std::vector<std::vector<O>> dp(std::size_t(n)+1,std::vector<O>(n));
    for (auto& x:dp[0]) x=0;
    for (int k=1; k<=n; ++k) for (auto e:g.edges()) if (dp[k-1][e.from]) {
        W x=optimization_detail::add(*dp[k-1][e.from],W(e.cost));
        auto& y=dp[k][e.to]; if (!y || x<*y) y=x;
    }
    auto less=[](W a,int b,W c,int d) {return optimization_detail::mul(a,d)<optimization_detail::mul(c,b);};
    std::optional<MinimumMeanCycle> best;
    for (int v=0; v<n; ++v) if (dp[n][v]) {
        O num; int den=1;
        for (int k=0; k<n; ++k) if (dp[k][v]) {
            W a=optimization_detail::sub(*dp[n][v],*dp[k][v]); int b=n-k;
            if (!num || less(*num,den,a,b)) {num=a;den=b;}
        }
        if (!best || less(*num,den,best->numerator,best->denominator)) best=MinimumMeanCycle{*num,den,{},{}};
    }
    if (!best) return std::nullopt;
    int divisor=std::gcd(int(best->numerator%best->denominator),best->denominator);
    best->numerator/=divisor; best->denominator/=divisor;
    // Reweight by the optimal mean. Tight arcs contain a zero-weight cycle.
    std::vector<W> weight(g.edge_count()),potential(n);
    for (auto e:g.edges()) weight[e.idx]=optimization_detail::sub(optimization_detail::mul(W(e.cost),best->denominator),best->numerator);
    for (int k=1; k<n; ++k) {
        bool changed=false;
        for (auto e:g.edges()) {
            W x=optimization_detail::add(potential[e.from],weight[e.idx]);
            if (x<potential[e.to]) {potential[e.to]=x;changed=true;}
        }
        if (!changed) break;
    }
    Graph<int> tight(n,GraphKind::Directed); std::vector<int> ids;
    for (auto e:g.edges()) if (potential[e.to]==optimization_detail::add(potential[e.from],weight[e.idx])) {
        tight.add_edge(e.from,e.to); ids.push_back(e.idx);
    }
    auto cycle=find_cycle(tight);
    if (!cycle) throw std::logic_error("minimum mean tight cycle missing");
    best->vertices=cycle->vertices; best->vertices.push_back(best->vertices.front());
    for (int id:cycle->edges) best->edge_ids.push_back(ids[id]);
    return best;
}
} // namespace cp
