#pragma once
// 説明: docs/graph_basic/shortest_directed_cycle.md
#include <optional>
#include <stdexcept>
#include "dijkstra.hpp"
#include "weighted_path.hpp"

namespace cp {
template<class T> std::optional<WeightedPath<T>> shortest_directed_cycle(const Graph<T>& g) {
    if(!g.directed()) throw std::invalid_argument("directed cycle requires directed graph");
    for(const auto& e:g.edges()) if(e.cost<T(0)) throw std::invalid_argument("nonnegative cycle weights required");
    std::optional<WeightedPath<T>> best;
    for(int s=0;s<g.size();++s) {
        auto paths=dijkstra(g,s);
        for(const auto& edge:g.edges()) if(edge.to==s && paths.reachable(edge.from)) {
            T cost=detail::distance_add(paths.dist[edge.from],edge.cost);
            if(best && !(cost<best->cost)) continue;
            auto vertices=paths.vertex_path(edge.from),edges=paths.edge_path(edge.from); vertices.push_back(s); edges.push_back(edge.idx);
            best=WeightedPath<T>{cost,std::move(vertices),std::move(edges)};
        }
    }
    return best;
}
} // namespace cp
