#pragma once
// 説明: docs/graph_basic/second_best_mst.md
#include <optional>
#include "mst_sensitivity.hpp"
namespace cp {
// 同じ辺集合以外の最小の全域木。strict=trueなら総コストも厳密に大きい木。
template<class T> std::optional<MinimumSpanningForestResult<T>> second_best_spanning_tree(const Graph<T>& g,bool strict=false) {
    auto analysis=mst_sensitivity(g); if(!analysis) return std::nullopt;
    const auto& alternative=strict?analysis->strictly_larger:analysis->different;
    if(!alternative) return std::nullopt;
    MinimumSpanningForestResult<T> result; result.cost=alternative->cost; result.component_count=g.size()?1:0;
    for(const auto& e:g.edges()) if((analysis->in_tree[e.idx] && e.idx!=alternative->removed_edge) || e.idx==alternative->added_edge) result.edges.push_back(e);
    return result;
}
} // namespace cp
