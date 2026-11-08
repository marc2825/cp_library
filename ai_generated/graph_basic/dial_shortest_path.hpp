#pragma once
#include <cassert>
#include <cstddef>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <vector>
#include "graph.hpp"
#include "shortest_path.hpp"

namespace cp {
// 整数重み0..max_weight。循環するC+1個のバケット。O(V(C+1)+E)。
template<class T> ShortestPathResult<T> dial_shortest_path(const Graph<T>& g,int start,int max_weight) {
    static_assert(std::is_integral<T>::value && !std::is_same<T,bool>::value && sizeof(T)<=8,"integer weights required");
    assert(0<=start && start<g.size());
    if(max_weight<0 || max_weight>100000000) throw std::length_error("Dial bucket count out of range");
    for(const auto& e:g.edges()) { assert(e.cost>=T(0) && static_cast<unsigned long long>(e.cost)<=static_cast<unsigned>(max_weight)); (void)e; }
    const auto width=static_cast<unsigned long long>(max_weight)+1;
    std::vector<std::vector<std::pair<T,int>>> buckets(static_cast<std::size_t>(width));
    ShortestPathResult<T> result(g.size(),start); result.dist[start]=T(0);
    buckets[0].emplace_back(T(0),start); std::size_t pending=1; unsigned long long current=0;
    while(pending) {
        auto& bucket=buckets[current%width];
        if(bucket.empty()) { ++current; continue; }
        auto [d,v]=bucket.back(); bucket.pop_back(); --pending;
        if(d!=result.dist[v]) continue;
        for(const auto& e:g[v]) {
            T candidate=detail::distance_add(d,e.cost);
            if(candidate>=result.dist[e.to]) continue;
            result.dist[e.to]=candidate; result.parent[e.to]=v; result.parent_edge[e.to]=e.idx;
            buckets[static_cast<unsigned long long>(candidate)%width].emplace_back(candidate,e.to); ++pending;
        }
    }
    return result;
}
} // namespace cp
