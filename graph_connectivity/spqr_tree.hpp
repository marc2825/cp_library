#pragma once
// 説明: docs/graph_connectivity/spqr_tree.md
#include <algorithm>
#include <map>
#include <numeric>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>
#include "graph.hpp"
namespace cp {
struct SPQRTree {
    enum class Kind { S,P,R,Q };
    struct Edge { int from,to,original_edge=-1,virtual_id=-1,twin_node=-1,twin_edge=-1; };
    struct Node { Kind kind; std::vector<Edge> edges; };
    std::vector<Node> nodes;
    Graph<int> tree;
    std::vector<std::pair<int,int>> original_location;
};
// 分離対の全探索版。二重頂点連結・ループなし無向多重グラフ。Q は単一辺だけ。
// 線形時間版ではない: O(m n^2 (n+m)) 時間、O(n+m) 出力領域。
template<class T> std::optional<SPQRTree> spqr_tree(const Graph<T>& g) {
    if(g.directed()) throw std::invalid_argument("undirected SPQR graph required");
    int n=g.size();
    for(auto e:g.edges()) if(e.from==e.to) throw std::invalid_argument("SPQR loops unsupported");
    if(n<2 || g.edge_count()==0) return std::nullopt;
    // articulation と非連結を検査（多重辺を許す）。
    for(int removed=-1;removed<n;++removed) {
        int start=removed==0?1:0; std::vector<bool> seen(n); std::vector<int> queue{start}; seen[start]=true;
        for(std::size_t i=0;i<queue.size();++i) for(auto e:g[queue[i]]) if(e.to!=removed && !seen[e.to]) { seen[e.to]=true; queue.push_back(e.to); }
        if(int(queue.size())!=n-(removed>=0)) return std::nullopt;
    }
    using E=SPQRTree::Edge; using Kind=SPQRTree::Kind;
    SPQRTree r; std::vector<std::vector<E>> work(1); int virtual_count=0;
    for(auto e:g.edges()) work[0].push_back({e.from,e.to,e.idx,-1,-1,-1});
    while(!work.empty()) {
        auto edges=std::move(work.back()); work.pop_back(); std::vector<int> vertices,degree(n);
        for(auto e:edges) { ++degree[e.from]; ++degree[e.to]; }
        for(int v=0;v<n;++v) if(degree[v]) vertices.push_back(v);
        if(edges.size()==1) { r.nodes.push_back({Kind::Q,std::move(edges)}); continue; }
        if(vertices.size()==2) { r.nodes.push_back({Kind::P,std::move(edges)}); continue; }
        bool cycle=true; for(int v:vertices) if(degree[v]!=2) cycle=false;
        if(cycle) { r.nodes.push_back({Kind::S,std::move(edges)}); continue; }
        bool split=false;
        for(int u:vertices) {
            if(split) break;
            for(int v:vertices) if(u<v) {
                std::vector<std::vector<int>> adj(n); for(auto e:edges) if(e.from!=u && e.from!=v && e.to!=u && e.to!=v) { adj[e.from].push_back(e.to); adj[e.to].push_back(e.from); }
                std::vector<int> component(n,-1); int count=0;
                for(int s:vertices) if(s!=u && s!=v && component[s]<0) {
                    std::vector<int> queue{s}; component[s]=count;
                    for(std::size_t j=0;j<queue.size();++j) for(int w:adj[queue[j]]) if(component[w]<0) { component[w]=count; queue.push_back(w); }
                    ++count;
                }
                std::vector<std::vector<E>> parts(count);
                for(auto e:edges) {
                    if((e.from==u && e.to==v) || (e.from==v && e.to==u)) parts.push_back({e});
                    else { int w=(e.from==u || e.from==v)?e.to:e.from; parts[component[w]].push_back(e); }
                }
                if(parts.size()<2 || (parts.size()==2 && (parts[0].size()<2 || parts[1].size()<2))) continue;
                split=true;
                if(parts.size()==2) {
                    int id=virtual_count++; for(auto& p:parts) { p.push_back({u,v,-1,id,-1,-1}); work.push_back(std::move(p)); }
                } else {
                    std::vector<E> parallel;
                    for(auto& p:parts) {
                        if(p.size()==1) parallel.push_back(p.front());
                        else { int id=virtual_count++; E virtual_edge{u,v,-1,id,-1,-1}; parallel.push_back(virtual_edge); p.push_back(virtual_edge); work.push_back(std::move(p)); }
                    }
                    r.nodes.push_back({Kind::P,std::move(parallel)});
                }
                break;
            }
        }
        if(!split) r.nodes.push_back({Kind::R,std::move(edges)});
    }
    // S-S / P-P の隣接を併合し、標準的な reduced SPQR tree にする。
    int count=int(r.nodes.size()); std::vector<int> parent(count); std::iota(parent.begin(),parent.end(),0);
    auto root=[&](int v) { while(v!=parent[v]) { parent[v]=parent[parent[v]]; v=parent[v]; } return v; };
    std::vector<std::vector<std::pair<int,int>>> locations(virtual_count);
    for(int v=0;v<count;++v) for(int j=0;j<int(r.nodes[v].edges.size());++j) { int id=r.nodes[v].edges[j].virtual_id; if(id>=0) locations[id].emplace_back(v,j); }
    std::vector<bool> erased(virtual_count);
    for(int id=0;id<virtual_count;++id) {
        if(locations[id].size()!=2) throw std::logic_error("unpaired SPQR virtual edge");
        int a=locations[id][0].first,b=locations[id][1].first; auto kind=r.nodes[a].kind;
        if(kind==r.nodes[b].kind && (kind==Kind::S || kind==Kind::P)) { parent[root(a)]=root(b); erased[id]=true; }
    }
    std::vector<SPQRTree::Node> nodes; std::vector<int> index(count,-1);
    for(int v=0;v<count;++v) {
        int p=root(v); if(index[p]<0) { index[p]=int(nodes.size()); nodes.push_back({r.nodes[v].kind,{}}); }
        for(auto e:r.nodes[v].edges) if(e.virtual_id<0 || !erased[e.virtual_id]) nodes[index[p]].edges.push_back(e);
    }
    r.nodes=std::move(nodes); r.tree=Graph<int>(int(r.nodes.size())); r.original_location.resize(g.edge_count());
    for(auto& x:locations) x.clear();
    for(int v=0;v<int(r.nodes.size());++v) for(int j=0;j<int(r.nodes[v].edges.size());++j) {
        auto e=r.nodes[v].edges[j]; if(e.original_edge>=0) r.original_location[e.original_edge]={v,j}; else locations[e.virtual_id].emplace_back(v,j);
    }
    for(auto x:locations) if(!x.empty()) {
        auto [a,i]=x[0]; auto [b,j]=x[1]; r.nodes[a].edges[i].twin_node=b; r.nodes[a].edges[i].twin_edge=j;
        r.nodes[b].edges[j].twin_node=a; r.nodes[b].edges[j].twin_edge=i; r.tree.add_edge(a,b);
    }
    return r;
}
} // namespace cp
