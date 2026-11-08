#pragma once
#include <cassert>
#include <optional>
#include "shortest_path.hpp"
#include "topological_sort.hpp"

namespace cp {
// 負辺可。有向閉路があればnullopt。到達しない成分の閉路も拒否する。
template<class T> std::optional<ShortestPathResult<T>> dag_shortest_path(const Graph<T>& g,int start=0) {
    assert(g.directed() && 0<=start && start<g.size());
    auto order=topological_sort(g); if(!order) return std::nullopt;
    ShortestPathResult<T> result(g.size(),start); result.dist[start]=T(0);
    for(int v:*order) if(result.reachable(v)) for(const auto& e:g[v]) {
        T d=detail::distance_add(result.dist[v],e.cost);
        if(d>=result.dist[e.to]) continue;
        result.dist[e.to]=d; result.parent[e.to]=v; result.parent_edge[e.to]=e.idx;
    }
    return result;
}
} // namespace cp
