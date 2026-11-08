#pragma once
// 説明: docs/graph_flow/min_cost_circulation.md
#include "min_cost_b_flow.hpp"
namespace cp {
// 各頂点のnet outflow=0とするMinCostBFlow。負費用循環も最適化する。
template<class Cap=long long,class Cost=long long> class MinCostCirculation {
    MinCostBFlow<Cap,Cost> problem_;
public:
    using Result=typename MinCostBFlow<Cap,Cost>::Result;
    explicit MinCostCirculation(int n) : problem_(n) {}
    int add_edge(int from,int to,Cap lower,Cap upper,Cost cost) { return problem_.add_edge(from,to,lower,upper,cost); }
    std::optional<Result> solve() const { return problem_.solve(); }
};
} // namespace cp
