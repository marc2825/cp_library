#pragma once
// 説明: docs/graph_basic/minimum_diameter_spanning_tree.md
#include <algorithm>
#include <functional>
#include <optional>
#include <queue>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <vector>
#include "graph.hpp"
#include "../combinatorial_optimization/detail/arithmetic.hpp"
namespace cp {
struct MinimumDiameterSpanningTreeResult {
    OptimizationWide diameter=0,radius_twice=0,offset_twice=0;
    int center_vertex=-1,center_edge=-1;
    std::vector<int> edge_ids;
};
// 非負整数重み。連続中心からの最短路木、非連結なら nullopt。空グラフは直径0。
template<class T> std::optional<MinimumDiameterSpanningTreeResult> minimum_diameter_spanning_tree(const Graph<T>& g) {
    static_assert(std::is_integral_v<T> && sizeof(T)<=8 && !std::is_same_v<T,bool>);
    if(g.directed()) throw std::invalid_argument("undirected graph required");
    for(auto e:g.edges()) if(e.cost<T(0)) throw std::invalid_argument("negative tree weight");
    using W=OptimizationWide; using O=std::optional<W>; int n=g.size(); MinimumDiameterSpanningTreeResult r;
    if(n==0) return r;
    std::vector<std::vector<O>> d(n,std::vector<O>(n)); for(int v=0;v<n;++v) d[v][v]=0;
    for(auto e:g.edges()) if(!d[e.from][e.to] || W(e.cost)<*d[e.from][e.to]) d[e.from][e.to]=d[e.to][e.from]=W(e.cost);
    for(int k=0;k<n;++k) for(int i=0;i<n;++i) if(d[i][k]) for(int j=0;j<n;++j) if(d[k][j]) {
        W v=*d[i][k]+*d[k][j]; if(!d[i][j] || v<*d[i][j]) d[i][j]=v;
    }
    for(auto x:d[0]) if(!x) return std::nullopt;
    r.center_vertex=0; for(auto x:d[0]) r.radius_twice=std::max(r.radius_twice,2**x);
    for(int v=1;v<n;++v) { W ecc=0; for(auto x:d[v]) ecc=std::max(ecc,2**x); if(ecc<r.radius_twice) { r.radius_twice=ecc; r.center_vertex=v; } }
    for(auto e:g.edges()) if(e.from!=e.to) {
        int u=e.from,v=e.to; W length=e.cost;
        std::vector<std::pair<W,int>> order;
        for(int z=0;z<n;++z) order.emplace_back(*d[v][z]+length-*d[u][z],z);
        std::sort(order.begin(),order.end()); std::vector<W> pref(n+1,-1),suff(n+1,-1);
        for(int i=0;i<n;++i) pref[i+1]=std::max(pref[i],*d[v][order[i].second]);
        for(int i=n-1;i>=0;--i) suff[i]=std::max(suff[i+1],*d[u][order[i].second]);
        for(int i=0;i<=n;++i) {
            W lo=i?std::max(W(0),order[i-1].first):W(0),hi=i<n?std::min(2*length,order[i].first):2*length;
            if(lo>hi) continue;
            W x=i==0?lo:(i==n?hi:std::max(lo,std::min(hi,length+pref[i]-suff[i])));
            W ecc=0; if(i) ecc=std::max(ecc,2*length-x+2*pref[i]); if(i<n) ecc=std::max(ecc,x+2*suff[i]);
            if(ecc<r.radius_twice) { r.radius_twice=ecc; r.center_vertex=-1; r.center_edge=e.idx; r.offset_twice=x; }
        }
    }
    std::vector<O> distance(n); std::vector<int> parent_edge(n,-1);
    using Item=std::pair<W,int>; std::priority_queue<Item,std::vector<Item>,std::greater<Item>> heap;
    auto seed=[&](int v,W dist) { distance[v]=dist; heap.emplace(dist,v); };
    if(r.center_vertex>=0) seed(r.center_vertex,0);
    else { auto e=g.edges()[r.center_edge]; seed(e.from,r.offset_twice); seed(e.to,2*W(e.cost)-r.offset_twice); }
    while(!heap.empty()) {
        auto [dist,v]=heap.top(); heap.pop(); if(dist!=*distance[v]) continue;
        for(auto e:g[v]) { W nd=dist+2*W(e.cost); if(!distance[e.to] || nd<*distance[e.to]) { distance[e.to]=nd; parent_edge[e.to]=e.idx; heap.emplace(nd,e.to); } }
    }
    int roots=0; for(int id:parent_edge) { if(id>=0) r.edge_ids.push_back(id); else ++roots; }
    if(roots==2) r.edge_ids.push_back(r.center_edge);
    // 実際の返却木の直径も記録する（中心半径との一致は独立検証）。
    std::vector<std::vector<std::pair<int,W>>> tree(n);
    for(int id:r.edge_ids) { auto e=g.edges()[id]; tree[e.from].emplace_back(e.to,W(e.cost)); tree[e.to].emplace_back(e.from,W(e.cost)); }
    auto farthest=[&](int s) {
        std::vector<std::pair<int,int>> stack{{s,-1}}; std::vector<W> dist(n); int best=s;
        while(!stack.empty()) { auto [v,p]=stack.back(); stack.pop_back(); if(dist[v]>dist[best]) best=v; for(auto [u,w]:tree[v]) if(u!=p) { dist[u]=dist[v]+w; stack.emplace_back(u,v); } }
        return std::make_pair(best,dist[best]);
    };
    r.diameter=farthest(farthest(0).first).second; return r;
}
} // namespace cp
