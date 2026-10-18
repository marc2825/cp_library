#pragma once
// 説明: docs/graph_combinatorics/disjoint_paths.md
#include <algorithm>
#include <cassert>
#include <queue>
#include <stdexcept>
#include <vector>
#include "../graph/graph.hpp"
#include "../graph_flow/dinic.hpp"
namespace cp {
struct DisjointPathsResult { std::vector<std::vector<int>> paths,edge_ids; int size() const{return static_cast<int>(paths.size());} };
// vertex_disjoint=trueなら内部頂点素。端点は共有する。辺はどちらの版も共有しない。
template<class T> DisjointPathsResult disjoint_paths(const Graph<T>& g,int source,int sink,bool vertex_disjoint=false) {
    int n=g.size(); assert(0<=source && source<n && 0<=sink && sink<n); if(source==sink) throw std::invalid_argument("distinct path endpoints required");
    Dinic<int> flow(2*n); int limit=g.edge_count();
    for(int v=0;v<n;++v) flow.add_edge(2*v,2*v+1,vertex_disjoint && v!=source && v!=sink?1:limit);
    struct Arc {int from,to,id,network;}; std::vector<Arc> arcs;
    for(auto e:g.edges()) if(e.from!=e.to) {
        arcs.push_back({e.from,e.to,e.idx,flow.add_edge(2*e.from+1,2*e.to,1)});
        if(!g.directed()) arcs.push_back({e.to,e.from,e.idx,flow.add_edge(2*e.to+1,2*e.from,1)});
    }
    int count=flow.max_flow(2*source+1,2*sink,limit); std::vector<int> remaining(arcs.size());
    for(int i=0;i<static_cast<int>(arcs.size());++i) remaining[i]=flow.get_edge(arcs[i].network).flow;
    if(!g.directed()) for(int i=0;i<static_cast<int>(arcs.size());i+=2) {int cancel=std::min(remaining[i],remaining[i+1]);remaining[i]-=cancel;remaining[i+1]-=cancel;}
    std::vector<std::vector<int>> adj(n); for(int i=0;i<static_cast<int>(arcs.size());++i) adj[arcs[i].from].push_back(i);
    DisjointPathsResult result;
    for(int iteration=0;iteration<count;++iteration) {
        std::vector<int> parent(n,-1); parent[source]=-2; std::queue<int> q; q.push(source);
        while(!q.empty() && parent[sink]==-1) {int v=q.front();q.pop();for(int i:adj[v]) if(remaining[i] && parent[arcs[i].to]==-1) {parent[arcs[i].to]=i;q.push(arcs[i].to);}}
        if(parent[sink]==-1) throw std::logic_error("flow path decomposition");
        std::vector<int> vertices{sink},edges;
        for(int v=sink;v!=source;) {int i=parent[v];--remaining[i];edges.push_back(arcs[i].id);v=arcs[i].from;vertices.push_back(v);}
        std::reverse(vertices.begin(),vertices.end());std::reverse(edges.begin(),edges.end());result.paths.push_back(std::move(vertices));result.edge_ids.push_back(std::move(edges));
    }
    return result;
}
template<class T> DisjointPathsResult edge_disjoint_paths(const Graph<T>& g,int s,int t){return disjoint_paths(g,s,t,false);}
template<class T> DisjointPathsResult vertex_disjoint_paths(const Graph<T>& g,int s,int t){return disjoint_paths(g,s,t,true);}
} // namespace cp
