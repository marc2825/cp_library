#pragma once
// 説明: docs/graph_combinatorics/series_parallel_decomposition.md
#include <functional>
#include <map>
#include <optional>
#include <queue>
#include <stdexcept>
#include <vector>
#include "../graph/graph.hpp"
namespace cp {
struct SeriesParallelDecomposition {
    enum class Kind { edge,series,parallel };
    struct Node { Kind kind; int from,to,left=-1,right=-1,original_edge=-1; };
    int root=-1; std::vector<Node> nodes;
};
// 有向二端子直並列グラフの認識と式木。孤立点/ループ/逆向き辺は受理しない。
template<class T> std::optional<SeriesParallelDecomposition> series_parallel_decomposition(const Graph<T>& g,int source,int sink) {
    if(!g.directed()) throw std::invalid_argument("directed series-parallel graph required");
    int n=g.size(); if(source<0 || source>=n || sink<0 || sink>=n || source==sink) throw std::invalid_argument("invalid terminals");
    if(!g.edge_count()) return std::nullopt;
    SeriesParallelDecomposition r; using Kind=SeriesParallelDecomposition::Kind;
    std::vector<std::map<int,int>> out(n),in(n); std::vector<bool> used(n),removed(n); std::queue<int> queue;
    auto insert=[&](int u,int v,int id) {
        auto it=out[u].find(v);
        if(it!=out[u].end()) { int p=int(r.nodes.size()); r.nodes.push_back({Kind::parallel,u,v,it->second,id,-1}); id=p; }
        out[u][v]=in[v][u]=id; queue.push(u); queue.push(v);
    };
    for(auto e:g.edges()) {
        if(e.from==e.to) return std::nullopt;
        used[e.from]=used[e.to]=true; int id=int(r.nodes.size()); r.nodes.push_back({Kind::edge,e.from,e.to,-1,-1,e.idx}); insert(e.from,e.to,id);
    }
    for(bool x:used) if(!x) return std::nullopt;
    while(!queue.empty()) {
        int v=queue.front(); queue.pop();
        if(v==source || v==sink || removed[v] || in[v].size()!=1 || out[v].size()!=1) continue;
        auto [u,left]=*in[v].begin(); auto [w,right]=*out[v].begin(); if(u==w) return std::nullopt;
        int id=int(r.nodes.size()); r.nodes.push_back({Kind::series,u,w,left,right,-1});
        out[u].erase(v); in[w].erase(v); in[v].clear(); out[v].clear(); removed[v]=true; insert(u,w,id);
    }
    for(int v=0;v<n;++v) if(v!=source && v!=sink && !removed[v]) return std::nullopt;
    if(!in[source].empty() || !out[sink].empty() || out[source].size()!=1 || out[source].begin()->first!=sink) return std::nullopt;
    r.root=out[source].begin()->second; return r;
}
} // namespace cp
