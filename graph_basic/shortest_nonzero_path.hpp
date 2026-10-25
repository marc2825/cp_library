#pragma once
// 説明: docs/graph_basic/shortest_nonzero_path.md
#include <algorithm>
#include <functional>
#include <numeric>
#include <optional>
#include <queue>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <vector>
#include "graph.hpp"
#include "../combinatorial_optimization/detail/arithmetic.hpp"
namespace cp {
struct NonzeroPath { std::vector<int> vertices,edge_ids; };
struct ShortestNonzeroPathResult {
    using W=OptimizationWide;
    int source=-1;
    std::vector<std::optional<W>> distance;
    // 復元用の最短路木と blossom 証拠。利用者は変更しない。
    std::vector<int> parent,parent_edge,depth,witness,origin;
    std::vector<bool> ordinary;
    std::vector<std::pair<int,int>> endpoints;
    NonzeroPath path(int target) const {
        if(target<0 || target>=int(distance.size())) throw std::out_of_range("nonzero path target");
        if(!distance[target]) return {};
        auto tree=[&](int v) {
            NonzeroPath p; p.vertices.push_back(v);
            while(v!=source) { p.edge_ids.push_back(parent_edge[v]); v=parent[v]; p.vertices.push_back(v); }
            std::reverse(p.vertices.begin(),p.vertices.end()); std::reverse(p.edge_ids.begin(),p.edge_ids.end()); return p;
        };
        if(ordinary[target]) return tree(target);
        auto ancestor=[&](int a,int b) { while(depth[b]>depth[a]) b=parent[b]; return a==b; };
        std::vector<int> chain;
        for(int v=target;;v=origin[v]) { chain.push_back(v); if(origin[v]<0) break; }
        NonzeroPath p;
        for(auto it=chain.rbegin();it!=chain.rend();++it) {
            int w=*it,e=witness[w],u=endpoints[e].first,v=endpoints[e].second;
            if(origin[w]>=0) { u=origin[w]; v=endpoints[e].first^endpoints[e].second^u; }
            if(origin[w]<0) {
                if(!ancestor(w,v)) std::swap(u,v);
                p=tree(u); p.edge_ids.push_back(e); p.vertices.push_back(v);
                while(v!=w) { p.edge_ids.push_back(parent_edge[v]); v=parent[v]; p.vertices.push_back(v); }
            } else if(ancestor(w,v) && !ancestor(w,u)) {
                p.edge_ids.push_back(e); p.vertices.push_back(v);
                while(v!=w) { p.edge_ids.push_back(parent_edge[v]); v=parent[v]; p.vertices.push_back(v); }
            } else {
                NonzeroPath next=tree(v); next.edge_ids.push_back(e); next.vertices.push_back(u);
                int j=int(p.vertices.size())-1;
                while(p.vertices[j]!=w) { next.edge_ids.push_back(p.edge_ids[j-1]); next.vertices.push_back(p.vertices[--j]); }
                p=std::move(next);
            }
        }
        return p;
    }
};
// Group: value_type, identity(), op(a,b), inverse(a)。元辺方向のラベルを指定。
// 無向、非負整数重み、任意の（非可換でもよい）群。空の s-s path は非零ではない。
template<class Group,class T>
ShortestNonzeroPathResult shortest_nonzero_paths(const Graph<T>& g,const std::vector<typename Group::value_type>& labels,int source=0) {
    static_assert(std::is_integral_v<T> && sizeof(T)<=8 && !std::is_same_v<T,bool>);
    if(g.directed() || labels.size()!=g.edges().size()) throw std::invalid_argument("undirected labeled graph required");
    int n=g.size(); if(source<0 || source>=n) throw std::out_of_range("nonzero path source");
    using W=OptimizationWide; using L=typename Group::value_type;
    for(auto e:g.edges()) if(e.cost<T(0)) throw std::invalid_argument("negative path length");
    ShortestNonzeroPathResult r; r.source=source; r.parent.assign(n,-1); r.parent_edge.assign(n,-1); r.depth.resize(n);
    r.witness.assign(n,-1); r.origin.assign(n,-1); r.ordinary.resize(n); r.distance.resize(n);
    for(auto e:g.edges()) r.endpoints.emplace_back(e.from,e.to);
    std::vector<std::optional<W>> d(n),q(n); std::vector<L> label(n,Group::identity());
    using Item=std::pair<W,int>; std::priority_queue<Item,std::vector<Item>,std::greater<Item>> heap;
    d[source]=0; heap.emplace(0,source);
    while(!heap.empty()) {
        auto [length,v]=heap.top(); heap.pop(); if(length!=*d[v]) continue;
        for(auto e:g[v]) {
            W nd=optimization_detail::add(length,W(e.cost));
            if(!d[e.to] || nd<*d[e.to]) {
                d[e.to]=nd; r.parent[e.to]=v; r.parent_edge[e.to]=e.idx; r.depth[e.to]=r.depth[v]+1;
                L edge_label=g.edges()[e.idx].from==v?labels[e.idx]:Group::inverse(labels[e.idx]);
                label[e.to]=Group::op(label[v],edge_label); heap.emplace(nd,e.to);
            }
        }
    }
    std::vector<bool> consistent(g.edge_count()); std::vector<std::optional<W>> height(g.edge_count());
    std::vector<int> owner(g.edge_count(),-1),dsu(n); std::iota(dsu.begin(),dsu.end(),0);
    auto root=[&](int v) { int a=v; while(dsu[a]!=a) a=dsu[a]; while(dsu[v]!=v) { int p=dsu[v]; dsu[v]=a; v=p; } return a; };
    for(auto e:g.edges()) if(d[e.from] && d[e.to]) {
        consistent[e.idx]=Group::op(label[e.from],labels[e.idx])==label[e.to];
        if(!consistent[e.idx] && e.from!=e.to) {
            height[e.idx]=optimization_detail::add(optimization_detail::add(*d[e.from],*d[e.to]),W(e.cost)); heap.emplace(*height[e.idx],e.idx);
        }
    }
    while(!heap.empty()) {
        auto [h,id]=heap.top(); heap.pop(); if(h!=*height[id]) continue;
        auto e=g.edges()[id]; int a=root(e.from),b=root(e.to); std::vector<int> blossom;
        while(a!=b) {
            if(r.depth[a]<r.depth[b]) std::swap(a,b);
            blossom.push_back(a); a=root(r.parent[a]);
        }
        int from=owner[id];
        // 全頂点の q と証拠を先に確定し、この blossom の更新中に owner を上書きさせない。
        for(int w:blossom) { dsu[w]=a; q[w]=h-*d[w]; r.witness[w]=id; r.origin[w]=from; }
        for(int w:blossom) for(auto f:g[w]) if(consistent[f.idx]) {
            W candidate=optimization_detail::add(optimization_detail::add(*q[w],*d[f.to]),W(f.cost));
            if(!height[f.idx] || candidate<*height[f.idx]) { height[f.idx]=candidate; owner[f.idx]=w; heap.emplace(candidate,f.idx); }
        }
    }
    for(int v=0;v<n;++v) if(d[v]) {
        r.ordinary[v]=!(label[v]==Group::identity()); r.distance[v]=r.ordinary[v]?d[v]:q[v];
    }
    return r;
}
} // namespace cp
