#pragma once
// 説明: docs/graph_connectivity/chordal_graph.md
#include <algorithm>
#include <array>
#include <queue>
#include <stdexcept>
#include <utility>
#include <vector>
#include "graph.hpp"

namespace cp {
struct ChordalGraphResult {
    bool is_chordal=true;
    std::vector<int> perfect_elimination_order,color,maximum_clique;
    int chromatic_number=0;
    std::array<int,3> violation{-1,-1,-1};
};
// MCS + 後の隣接頂点がcliqueかを検証。simple undirected graph。
template<class T> ChordalGraphResult chordal_graph(const Graph<T>& g) {
    if(g.directed()) throw std::invalid_argument("chordal graph requires undirected graph");
    int n=g.size(); std::vector<std::vector<int>> adjacency(n);
    for(int v=0;v<n;++v) {
        for(const auto& e:g[v]) adjacency[v].push_back(e.to);
        auto& row=adjacency[v]; std::sort(row.begin(),row.end());
        if(std::binary_search(row.begin(),row.end(),v) || std::adjacent_find(row.begin(),row.end())!=row.end()) throw std::invalid_argument("simple graph required");
    }
    std::priority_queue<std::pair<int,int>> queue; std::vector<int> weight(n),position(n,-1),order;
    for(int v=0;v<n;++v) queue.emplace(0,-v);
    while(!queue.empty()) {
        auto [w,negative]=queue.top(); queue.pop(); int v=-negative;
        if(position[v]>=0 || w!=weight[v]) continue;
        position[v]=int(order.size()); order.push_back(v);
        for(int u:adjacency[v]) if(position[u]<0) queue.emplace(++weight[u],-u);
    }
    std::reverse(order.begin(),order.end()); for(int i=0;i<n;++i) position[order[i]]=i;
    ChordalGraphResult result; result.perfect_elimination_order=order;
    for(int v:order) {
        int parent=-1; std::vector<int> later;
        for(int u:adjacency[v]) if(position[u]>position[v]) { later.push_back(u); if(parent<0 || position[u]<position[parent]) parent=u; }
        for(int u:later) if(u!=parent && !std::binary_search(adjacency[parent].begin(),adjacency[parent].end(),u)) {
            result.is_chordal=false; result.violation={v,parent,u}; return result;
        }
        later.push_back(v); if(later.size()>result.maximum_clique.size()) result.maximum_clique=std::move(later);
    }
    result.color.assign(n,-1); std::vector<int> used(n,-1);
    for(auto it=order.rbegin();it!=order.rend();++it) {
        int v=*it; for(int u:adjacency[v]) if(result.color[u]>=0) used[result.color[u]]=v;
        int c=0; while(c<n && used[c]==v) ++c;
        result.color[v]=c; result.chromatic_number=std::max(result.chromatic_number,c+1);
    }
    return result;
}
} // namespace cp
