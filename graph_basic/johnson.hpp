#pragma once
#include <optional>
#include <type_traits>
#include <vector>
#include "dijkstra.hpp"

namespace cp {
// 符号付き整数の全点対最短路。どこかに負閉路があればnullopt。
template<class T> std::optional<std::vector<ShortestPathResult<T>>> johnson(const Graph<T>& g) {
    static_assert(std::is_integral<T>::value && std::is_signed<T>::value,"signed integer weights required");
    const int n=g.size(); std::vector<T> h(n,T(0));
    for(int round=0;round<n;++round) {
        bool changed=false;
        for(int u=0;u<n;++u) for(const auto& e:g[u]) {
            T candidate=detail::distance_add(h[u],e.cost);
            if(candidate<h[e.to]) { h[e.to]=candidate; changed=true; }
        }
        if(!changed) break;
        if(round==n-1) return std::nullopt;
    }
    Graph<T> reduced(n,g.kind());
    for(const auto& e:g.edges()) reduced.add_edge(e.from,e.to,
        detail::distance_subtract(detail::distance_add(e.cost,h[e.from]),h[e.to]));
    std::vector<ShortestPathResult<T>> result; result.reserve(n);
    for(int s=0;s<n;++s) {
        auto row=dijkstra(reduced,s);
        for(int v=0;v<n;++v) if(row.reachable(v))
            row.dist[v]=detail::distance_add(detail::distance_subtract(row.dist[v],h[s]),h[v]);
        result.push_back(std::move(row));
    }
    return result;
}
} // namespace cp
