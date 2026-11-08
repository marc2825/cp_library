#pragma once
// 説明: docs/graph_basic/directed_mst.md
#include <algorithm>
#include <cassert>
#include <numeric>
#include <optional>
#include <stdexcept>
#include <vector>
#include "graph.hpp"
#include "../graph/detail/distance.hpp"

namespace cp {
template<class T> struct DirectedMSTResult {
    T cost=T(0);
    std::vector<int> parent,parent_edge;
};
// Chu–Liu/Edmonds。各縮約段を保存して元の辺を復元。O(VE)、負辺可。
template<class T> std::optional<DirectedMSTResult<T>> directed_mst(const Graph<T>& graph,int root=0) {
    if(!graph.directed()) throw std::invalid_argument("directed MST requires a directed graph");
    const int original_n=graph.size();
    if(!original_n) return DirectedMSTResult<T>{};
    assert(0<=root && root<original_n);
    struct Arc { int from,to,previous; T cost; };
    struct Layer { int n,root; std::vector<Arc> edges; std::vector<int> incoming; };
    std::vector<Layer> layers;
    Layer current{original_n,root,{},{}};
    for(const auto& e:graph.edges()) current.edges.push_back({e.from,e.to,e.idx,e.cost});
    std::vector<int> chosen;
    for(;;) {
        current.incoming.assign(current.n,-1);
        for(int i=0;i<int(current.edges.size());++i) {
            const auto& e=current.edges[i];
            if(e.from==e.to || e.to==current.root) continue;
            int& best=current.incoming[e.to];
            if(best<0 || e.cost<current.edges[best].cost) best=i;
        }
        for(int v=0;v<current.n;++v) if(v!=current.root && current.incoming[v]<0) return std::nullopt;
        std::vector<int> component(current.n,-1),seen(current.n,-1); int count=0;
        for(int s=0;s<current.n;++s) {
            int v=s;
            while(v!=current.root && component[v]<0 && seen[v]!=s) {
                seen[v]=s; v=current.edges[current.incoming[v]].from;
            }
            if(v!=current.root && component[v]<0) {
                int u=v;
                do { component[u]=count; u=current.edges[current.incoming[u]].from; } while(u!=v);
                ++count;
            }
        }
        if(!count) { chosen=current.incoming; break; }
        for(int v=0;v<current.n;++v) if(component[v]<0) component[v]=count++;
        Layer next{count,component[current.root],{},{}};
        for(int i=0;i<int(current.edges.size());++i) {
            const auto& e=current.edges[i]; int a=component[e.from],b=component[e.to];
            if(a==b || e.to==current.root) continue;
            next.edges.push_back({a,b,i,detail::distance_subtract(e.cost,current.edges[current.incoming[e.to]].cost)});
        }
        layers.push_back(std::move(current)); current=std::move(next);
    }
    while(!layers.empty()) {
        Layer previous=std::move(layers.back()); layers.pop_back();
        auto expanded=previous.incoming;
        for(int edge:chosen) if(edge>=0) {
            int id=current.edges[edge].previous; expanded[previous.edges[id].to]=id;
        }
        chosen=std::move(expanded); current=std::move(previous);
    }
    DirectedMSTResult<T> answer; answer.parent.assign(original_n,-1); answer.parent_edge.assign(original_n,-1); answer.parent[root]=root;
    for(int v=0;v<original_n;++v) if(v!=root) {
        int id=current.edges[chosen[v]].previous; const auto& e=graph.edges()[id];
        answer.parent[v]=e.from; answer.parent_edge[v]=id; answer.cost=detail::distance_add(answer.cost,e.cost);
    }
    return answer;
}
} // namespace cp
