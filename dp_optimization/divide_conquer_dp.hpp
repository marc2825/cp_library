#pragma once
#include <algorithm>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>
#include "monotone_minima.hpp"
#include "detail/arithmetic.hpp"

namespace cp {
struct DPStageResult {
    std::vector<std::optional<long long>> values;
    std::vector<int> parent;
};
// next[r]=min_{0<=l<r}(previous[l]+cost(l,r))。最左argminが非減少であること。
template<class Cost> DPStageResult divide_conquer_dp_stage(
        const std::vector<std::optional<long long>>& previous, Cost cost) {
    if (previous.empty()) throw std::invalid_argument("DP stage needs index zero");
    const int n = static_cast<int>(previous.size()) - 1;
    DPStageResult result{std::vector<std::optional<long long>>(n+1), std::vector<int>(n+1,-1)};
    struct Cell {
        std::optional<long long> value;
        bool operator<(const Cell& other) const {
            return value && (!other.value || *value < *other.value);
        }
    };
    auto get = [&](int row,int l)->Cell {
        const int r=row+1;
        if (l>=r || !previous[l]) return {std::nullopt};
        return {detail::dp_add(*previous[l],cost(l,r))};
    };
    auto best=monotone_minima(n,n,get);
    for (int r=1;r<=n;++r) {
        result.values[r]=get(r-1,best[r-1]).value;
        if (result.values[r]) result.parent[r]=best[r-1];
    }
    return result;
}
struct DivideConquerDPResult {
    std::vector<std::vector<std::optional<long long>>> dp;
    std::vector<std::vector<int>> parent;
    std::optional<std::vector<std::pair<int,int>>> partitions(int groups,int end) const {
        if (groups<0 || groups>=static_cast<int>(dp.size()) || end<0 || end>=static_cast<int>(dp[0].size()))
            throw std::out_of_range("partition reconstruction outside DP table");
        if (!dp[groups][end]) return std::nullopt;
        std::vector<std::pair<int,int>> answer;
        for (int k=groups;k>0;--k) { int l=parent[k][end]; answer.emplace_back(l,end); end=l; }
        std::reverse(answer.begin(),answer.end()); return answer;
    }
};
// [0,n)をちょうどgroups個の非空連続区間に分割。O(groups*n*log(n+1))評価。
template<class Cost> DivideConquerDPResult divide_conquer_dp(int n,int groups,Cost cost) {
    if (n<0 || groups<0) throw std::invalid_argument("negative partition dimension");
    DivideConquerDPResult result;
    result.dp.emplace_back(n+1); result.dp[0][0]=0; result.parent.emplace_back(n+1,-1);
    for (int k=0;k<groups;++k) {
        auto stage=divide_conquer_dp_stage(result.dp.back(),cost);
        result.dp.push_back(std::move(stage.values)); result.parent.push_back(std::move(stage.parent));
    }
    return result;
}
} // namespace cp
