#pragma once
// 説明: docs/graph_basic/shortest_undirected_cycle.md
#include <functional>
#include <optional>
#include <queue>
#include <stdexcept>
#include <utility>
#include "shortest_path.hpp"
#include "weighted_path.hpp"
#include "graph.hpp"

namespace cp {
// 各辺を一時的に除外して両端の最短路を求める。O(E(V+E)log(V+E))。
template<class T> std::optional<WeightedPath<T>> shortest_undirected_cycle(const Graph<T>& g) {
    if(g.directed()) throw std::invalid_argument("undirected cycle requires undirected graph");
    for(const auto& e:g.edges()) if(e.cost<T(0)) throw std::invalid_argument("nonnegative cycle weights required");
    std::optional<WeightedPath<T>> best;
    for(const auto& excluded:g.edges()) {
        ShortestPathResult<T> paths(g.size(),excluded.from); paths.dist[excluded.from]=T(0);
        using Item=std::pair<T,int>;
        std::priority_queue<Item,std::vector<Item>,std::greater<Item>> queue; queue.emplace(T(0),excluded.from);
        while(!queue.empty()) {
            auto [distance,v]=queue.top(); queue.pop(); if(distance!=paths.dist[v]) continue;
            if(v==excluded.to) break;
            for(const auto& e:g[v]) if(e.idx!=excluded.idx) {
                T next=detail::distance_add(distance,e.cost);
                if(next<paths.dist[e.to]) { paths.dist[e.to]=next; paths.parent[e.to]=v; paths.parent_edge[e.to]=e.idx; queue.emplace(next,e.to); }
            }
        }
        if(!paths.reachable(excluded.to)) continue;
        T cost=detail::distance_add(paths.dist[excluded.to],excluded.cost);
        if(best && !(cost<best->cost)) continue;
        auto vertices=paths.vertex_path(excluded.to),edges=paths.edge_path(excluded.to); vertices.push_back(excluded.from); edges.push_back(excluded.idx);
        best=WeightedPath<T>{cost,std::move(vertices),std::move(edges)};
    }
    return best;
}
} // namespace cp
