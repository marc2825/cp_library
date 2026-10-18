#pragma once
// 説明: docs/graph_connectivity/three_edge_connected_components.md
#include <algorithm>
#include <cstddef>
#include <stdexcept>
#include <utility>
#include <vector>
#include "low_link.hpp"
#include "../union_find/disjoint_set_union.hpp"

namespace cp {
struct ThreeEdgeConnectedComponentsResult { std::vector<int> component; std::vector<std::vector<int>> groups; };
// path absorption法。Luzhiled (Unlicense) の方法を辺ID・非再帰DFSへ移植。
// https://ei1333.github.io/library/graph/connected-components/three-edge-connected-components.hpp
template<class T> ThreeEdgeConnectedComponentsResult three_edge_connected_components(const Graph<T>& g) {
    if(g.directed()) throw std::invalid_argument("three-edge components require undirected graph");
    int n=g.size(); auto bridges=low_link(g); std::vector<bool> excluded(g.edge_count());
    for(int id:bridges.bridge_edges) excluded[id]=true;
    DisjointSetUnion dsu(n); std::vector<int> in(n,-1),out(n),low(n),parent(n,-1),parent_edge(n,-1);
    std::vector<long long> degree(n); std::vector<std::vector<int>> paths(n); int timer=0;
    auto absorb=[&](std::vector<int>& path,int v,int descendant=-1) {
        while(!path.empty()) {
            int x=path.back();
            if(descendant>=0 && !(in[x]<=in[descendant] && in[descendant]<out[x])) break;
            path.pop_back(); dsu.merge(v,x); degree[v]+=degree[x]-2;
        }
    };
    for(int start=0;start<n;++start) if(in[start]<0) {
        in[start]=low[start]=timer++; std::vector<std::pair<int,std::size_t>> stack{{start,0}};
        while(!stack.empty()) {
            int v=stack.back().first; auto& next=stack.back().second;
            if(next<g[v].size()) {
                const auto e=g[v][next++]; int u=e.to;
                if(u==v || excluded[e.idx]) continue;
                ++degree[v]; if(e.idx==parent_edge[v]) continue;
                if(in[u]<0) { parent[u]=v; parent_edge[u]=e.idx; in[u]=low[u]=timer++; stack.emplace_back(u,0); }
                else if(in[u]<in[v]) { if(in[u]<low[v]) { low[v]=in[u]; absorb(paths[v],v); } }
                else { degree[v]-=2; absorb(paths[v],v,u); }
            } else {
                out[v]=timer; paths[v].push_back(v); stack.pop_back(); int p=parent[v];
                if(p<0) continue;
                if(degree[v]==2) paths[v].pop_back();
                if(low[v]<low[p]) { low[p]=low[v]; absorb(paths[p],p); paths[p]=std::move(paths[v]); }
                else absorb(paths[v],p);
            }
        }
    }
    ThreeEdgeConnectedComponentsResult result; result.groups=dsu.groups(); result.component.resize(n);
    for(int i=0;i<int(result.groups.size());++i) for(int v:result.groups[i]) result.component[v]=i;
    return result;
}
} // namespace cp
