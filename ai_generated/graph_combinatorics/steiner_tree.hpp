#pragma once
// 説明: docs/graph_combinatorics/steiner_tree.md
#include <algorithm>
#include <cstddef>
#include <functional>
#include <optional>
#include <queue>
#include <stdexcept>
#include <utility>
#include <vector>
#include "../graph/graph.hpp"
#include "../graph/detail/distance.hpp"
#include "../union_find/disjoint_set_union.hpp"
namespace cp {
template<class T> struct SteinerTreeResult { T cost=T(0); std::vector<int> vertices,edge_ids; };
// 非負辺の無向グラフ。2^K V状態を確保するDreyfus-Wagner法。
template<class T> std::optional<SteinerTreeResult<T>> steiner_tree(const Graph<T>& g,std::vector<int> terminals) {
    if(g.directed()) throw std::invalid_argument("Steiner tree requires undirected graph");
    int n=g.size(); for(int v:terminals) if(v<0 || v>=n) throw std::out_of_range("terminal");
    for(auto e:g.edges()) if(e.cost<T(0)) throw std::invalid_argument("negative Steiner edge");
    std::sort(terminals.begin(),terminals.end()); terminals.erase(std::unique(terminals.begin(),terminals.end()),terminals.end());
    SteinerTreeResult<T> result; int k=static_cast<int>(terminals.size()); if(k==0) return result;
    if(k>20) throw std::length_error("Steiner tree requires at most 20 distinct terminals");
    std::size_t masks=std::size_t(1)<<k;
    if(static_cast<std::size_t>(n)>std::vector<T>().max_size()/masks) throw std::length_error("Steiner DP too large");
    struct Parent { int split=0,vertex=-1,edge=-1; };
    const T inf=distance_inf<T>(); std::vector<T> dp(masks*n,inf); std::vector<Parent> parent(masks*n);
    for(int i=0;i<k;++i) dp[(std::size_t(1)<<i)*n+terminals[i]]=T(0);
    using Item=std::pair<T,int>;
    for(std::size_t mask=1;mask<masks;++mask) {
        auto bit=mask&(~mask+1);
        for(auto sub=(mask-1)&mask;sub;sub=(sub-1)&mask) if(sub&bit) {
            auto other=mask^sub; if(!other) continue;
            for(int v=0;v<n;++v) if(dp[sub*n+v]!=inf && dp[other*n+v]!=inf) {
                T candidate=detail::distance_add(dp[sub*n+v],dp[other*n+v]);
                if(candidate<dp[mask*n+v]) { dp[mask*n+v]=candidate; parent[mask*n+v]={static_cast<int>(sub),-1,-1}; }
            }
        }
        std::priority_queue<Item,std::vector<Item>,std::greater<Item>> queue;
        for(int v=0;v<n;++v) if(dp[mask*n+v]!=inf) queue.emplace(dp[mask*n+v],v);
        while(!queue.empty()) {
            auto [d,v]=queue.top(); queue.pop(); if(d!=dp[mask*n+v]) continue;
            for(auto e:g[v]) { T nd=detail::distance_add(d,e.cost); if(nd<dp[mask*n+e.to]) { dp[mask*n+e.to]=nd; parent[mask*n+e.to]={0,v,e.idx}; queue.emplace(nd,e.to); } }
        }
    }
    std::size_t full=masks-1; int best=static_cast<int>(std::min_element(dp.begin()+full*n,dp.end())-(dp.begin()+full*n));
    if(dp[full*n+best]==inf) return std::nullopt;
    std::vector<bool> seen(dp.size()),used(g.edge_count()); std::vector<std::pair<std::size_t,int>> stack{{full,best}};
    while(!stack.empty()) {
        auto [mask,v]=stack.back(); stack.pop_back(); auto id=mask*n+v; if(seen[id]) continue; seen[id]=true; auto p=parent[id];
        if(p.split) { stack.emplace_back(p.split,v); stack.emplace_back(mask^p.split,v); }
        else if(p.edge!=-1) { used[p.edge]=true; stack.emplace_back(mask,p.vertex); }
    }
    // ゼロ重みの重複経路も含め、辺集合を木にして非terminal葉を除く。
    DisjointSetUnion dsu(n); std::vector<std::vector<std::pair<int,int>>> adj(n); std::vector<bool> alive(g.edge_count()),terminal(n);
    for(int v:terminals) terminal[v]=true;
    auto edges=g.edges(); for(auto e:edges) if(used[e.idx] && !dsu.same(e.from,e.to)) { dsu.merge(e.from,e.to); alive[e.idx]=true; adj[e.from].emplace_back(e.to,e.idx); adj[e.to].emplace_back(e.from,e.idx); }
    std::queue<int> leaves; std::vector<int> degree(n); for(int v=0;v<n;++v) { degree[v]=static_cast<int>(adj[v].size()); if(!terminal[v] && degree[v]==1) leaves.push(v); }
    while(!leaves.empty()) { int v=leaves.front(); leaves.pop(); for(auto [u,id]:adj[v]) if(alive[id]) { alive[id]=false; --degree[v]; if(--degree[u]==1 && !terminal[u]) leaves.push(u); } }
    std::vector<bool> present=terminal;
    for(auto e:edges) if(alive[e.idx]) { result.edge_ids.push_back(e.idx); present[e.from]=present[e.to]=true; result.cost=detail::distance_add(result.cost,e.cost); }
    for(int v=0;v<n;++v) if(present[v]) result.vertices.push_back(v);
    return result;
}
} // namespace cp
