#pragma once
#include <algorithm>
#include <cassert>
#include <functional>
#include <queue>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>
#include "dijkstra.hpp"
#include "weighted_path.hpp"
#include "../ordered_structure/persistent_leftist_heap.hpp"

namespace cp {
template<class T> struct KShortestWalksResult {
    struct Choice { int previous,from,to,edge; };
    int source=-1,target=-1;
    std::vector<T> costs;
    std::vector<int> successor,tree_edge,tails;
    std::vector<Choice> choices;
    // 復元は出力長に比例。巨大なwalkを必要なときだけ展開する。
    WeightedPath<T> path(int rank) const {
        assert(0<=rank && rank<int(costs.size()));
        std::vector<int> chain;
        for(int c=tails[rank];c!=-1;c=choices[c].previous) chain.push_back(c);
        std::reverse(chain.begin(),chain.end());
        WeightedPath<T> result{costs[rank],{source},{}}; int v=source;
        auto follow=[&](int stop) {
            while(v!=stop) { assert(successor[v]!=-1); result.edge_ids.push_back(tree_edge[v]); v=successor[v]; result.vertices.push_back(v); }
        };
        for(int c:chain) { const auto& arc=choices[c]; follow(arc.from); result.edge_ids.push_back(arc.edge); v=arc.to; result.vertices.push_back(v); }
        follow(target); return result;
    }
};

// Eppstein法：最短路木上のsidetrackを永続leftist heapで共有する。
template<class T> KShortestWalksResult<T> k_shortest_walks(const Graph<T>& g,int s,int t,int k) {
    static_assert(std::is_integral<T>::value && !std::is_same<T,bool>::value,"integer weights required");
    assert(0<=s && s<g.size() && 0<=t && t<g.size() && k>=0);
    for(const auto& e:g.edges()) { assert(e.cost>=T(0)); (void)e; }
    KShortestWalksResult<T> result; result.source=s; result.target=t; if(!k) return result;
    Graph<T> reverse(g.size(),g.kind());
    for(const auto& e:g.edges()) reverse.add_edge(e.to,e.from,e.cost);
    auto paths=dijkstra(reverse,t); result.successor=paths.parent; result.tree_edge=paths.parent_edge;
    if(!paths.reachable(s)) return result;
    struct Sidetrack { T delta; int from,to,edge; };
    struct Compare { bool operator()(const Sidetrack& a,const Sidetrack& b) const { return a.delta>b.delta; } };
    PersistentLeftistHeap<Sidetrack,Compare> pool;
    std::vector<int> heaps(g.size(),0); std::vector<std::vector<int>> children(g.size());
    for(int v=0;v<g.size();++v) if(paths.reachable(v)) {
        if(paths.parent[v]!=-1) children[paths.parent[v]].push_back(v);
        int previous_loop=-1;
        std::vector<Sidetrack> local;
        for(const auto& e:g[v]) {
            // 無向自己ループの隣接表の二つの出現は同じ論理辺として数える。
            if(!g.directed() && e.to==v) { if(e.idx==previous_loop) continue; previous_loop=e.idx; }
            if(!paths.reachable(e.to) || (e.idx==paths.parent_edge[v] && e.to==paths.parent[v])) continue;
            T delta=detail::distance_subtract(detail::distance_add(e.cost,paths.dist[e.to]),paths.dist[v]);
            local.push_back({delta,v,e.to,e.idx});
        }
        heaps[v]=pool.build(local.begin(),local.end());
    }
    std::vector<int> order{t};
    for(std::size_t i=0;i<order.size();++i) {
        int v=order[i]; if(paths.parent[v]!=-1) heaps[v]=pool.meld(heaps[v],heaps[paths.parent[v]]);
        order.insert(order.end(),children[v].begin(),children[v].end());
    }
    result.costs.push_back(paths.dist[s]); result.tails.push_back(-1);
    using Candidate=std::tuple<T,int,int>;
    std::priority_queue<Candidate,std::vector<Candidate>,std::greater<Candidate>> queue;
    if(k>1 && heaps[s]) queue.emplace(detail::distance_add(paths.dist[s],pool.top(heaps[s]).delta),heaps[s],-1);
    while(!queue.empty() && int(result.costs.size())<k) {
        auto [cost,node,prefix]=queue.top(); queue.pop(); const auto current=pool.top(node);
        int choice=int(result.choices.size()); result.choices.push_back({prefix,current.from,current.to,current.edge});
        result.costs.push_back(cost); result.tails.push_back(choice);
        if(int(result.costs.size())==k) break;
        auto [left,right]=pool.children(node);
        for(int child:{left,right}) if(child)
            queue.emplace(detail::distance_add(cost,detail::distance_subtract(pool.top(child).delta,current.delta)),child,prefix);
        int continuation=heaps[current.to];
        if(continuation) queue.emplace(detail::distance_add(cost,pool.top(continuation).delta),continuation,choice);
    }
    return result;
}
} // namespace cp
