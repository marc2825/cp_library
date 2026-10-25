#pragma once
// 説明: docs/graph_basic/few_weight_shortest_path.md
#include <deque>
#include <map>
#include <stdexcept>
#include <utility>
#include <vector>
#include "graph.hpp"
#include "shortest_path.hpp"
namespace cp {
// k 種類の非負重み。分類 O(m log(k+1))、探索 O(m+kn)。
template<class T> ShortestPathResult<T> few_weight_shortest_path(const Graph<T>& g,int source=0) {
    if(source<0 || source>=g.size()) throw std::out_of_range("shortest path source");
    std::map<T,int> classes;
    for(auto e:g.edges()) { if(!(e.cost>=T(0))) throw std::invalid_argument("nonnegative weights required"); classes.emplace(e.cost,0); }
    int k=0; for(auto& entry:classes) entry.second=k++;
    std::vector<int> type(g.edge_count()); for(auto e:g.edges()) type[e.idx]=classes.at(e.cost);
    using Item=std::pair<T,int>; std::vector<std::deque<Item>> queues(k);
    ShortestPathResult<T> r(g.size(),source); std::vector<bool> done(g.size()); r.dist[source]=T(0);
    int v=source;
    while(v!=-1) {
        done[v]=true;
        for(auto e:g[v]) {
            T d=detail::distance_add(r.dist[v],e.cost);
            if(d<r.dist[e.to]) { r.dist[e.to]=d; r.parent[e.to]=v; r.parent_edge[e.to]=e.idx; queues[type[e.idx]].emplace_back(d,e.to); }
        }
        v=-1; T best=distance_inf<T>();
        for(auto& q:queues) {
            while(!q.empty() && (done[q.front().second] || q.front().first!=r.dist[q.front().second])) q.pop_front();
            if(!q.empty() && q.front().first<best) { best=q.front().first; v=q.front().second; }
        }
    }
    return r;
}
} // namespace cp
