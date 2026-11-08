#pragma once
// 説明: docs/graph_flow/maximum_weight_bipartite_matching.md
#include <algorithm>
#include <limits>
#include <optional>
#include <stdexcept>
#include <tuple>
#include <utility>
#include <vector>
#include "../combinatorial_optimization/assignment_problem.hpp"

namespace cp {
struct MaximumWeightBipartiteMatchingResult { long long weight=0; std::vector<std::pair<int,int>> pairs; };
// 頂点を未使用にしてもよい重み最大化。Hungarianに左側個数の0費用dummy列を加える。
inline MaximumWeightBipartiteMatchingResult maximum_weight_bipartite_matching(int left_size,int right_size,const std::vector<std::tuple<int,int,long long>>& edges) {
    if(left_size<0 || right_size<0 || left_size>std::numeric_limits<int>::max()-right_size) throw std::invalid_argument("invalid bipartite dimensions");
    AssignmentProblem assignment(left_size,right_size+left_size);
    for(auto [u,v,w]:edges) {
        if(u<0 || u>=left_size || v<0 || v>=right_size) throw std::out_of_range("bipartite endpoint");
        auto previous=assignment.cost(u,v); if(!previous || *previous<w) assignment.set_cost(u,v,w);
    }
    for(int u=0;u<left_size;++u) for(int d=0;d<left_size;++d) assignment.set_cost(u,right_size+d,0);
    auto optimum=assignment.solve(AssignmentObjective::maximize); MaximumWeightBipartiteMatchingResult result; result.weight=optimum->cost;
    for(int u=0;u<left_size;++u) if(optimum->row_to_column[u]<right_size) result.pairs.emplace_back(u,optimum->row_to_column[u]);
    return result;
}
} // namespace cp
