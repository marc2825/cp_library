#pragma once
#include <cassert>
#include <type_traits>
#include "graph.hpp"
#include "shortest_path.hpp"
#include "../data_structure/radix_heap_iwiwi.hpp"

namespace cp {
// 非負整数重み。既存のiwiwi版Radix Heapを共有し、元の辺IDで復元する。
template<class T> ShortestPathResult<T> radix_heap_dijkstra(const Graph<T>& g,int start=0) {
    static_assert(std::is_integral<T>::value && !std::is_same<T,bool>::value && sizeof(T)<=8,"integer weights required");
    assert(0<=start && start<g.size());
    for(const auto& e:g.edges()) { assert(e.cost>=T(0)); (void)e; }
    using U=std::make_unsigned_t<T>;
    radix_heap::pair_radix_heap<U,int> queue;
    ShortestPathResult<T> result(g.size(),start); result.dist[start]=T(0); queue.push(U(0),start);
    while(!queue.empty()) {
        T d=static_cast<T>(queue.top_key()); int v=queue.top_value(); queue.pop();
        if(d!=result.dist[v]) continue;
        for(const auto& e:g[v]) {
            T candidate=detail::distance_add(d,e.cost);
            if(candidate>=result.dist[e.to]) continue;
            result.dist[e.to]=candidate; result.parent[e.to]=v; result.parent_edge[e.to]=e.idx;
            queue.push(static_cast<U>(candidate),e.to);
        }
    }
    return result;
}
} // namespace cp
