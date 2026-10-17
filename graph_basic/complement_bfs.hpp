#pragma once
#include <algorithm>
#include <cassert>
#include <numeric>
#include <vector>
#include "graph.hpp"
#include "shortest_path.hpp"

namespace cp {
// 補グラフの辺には元の辺IDがないため、復元は頂点列を返す。
struct ComplementBFSResult {
    int source; std::vector<int> dist,parent;
    bool reachable(int v) const { assert(0<=v && v<int(dist.size())); return dist[v]!=distance_inf<int>(); }
    std::vector<int> vertex_path(int v) const {
        if(!reachable(v)) return {};
        std::vector<int> path; for(;v!=-1;v=parent[v]) path.push_back(v);
        std::reverse(path.begin(),path.end()); return path;
    }
};
// 無向グラフ限定。自己ループを無視し、多重辺も可。未訪問リストでO(V+E)。
template<class T> ComplementBFSResult complement_bfs(const Graph<T>& g,int start=0) {
    assert(!g.directed() && 0<=start && start<g.size()); int n=g.size();
    ComplementBFSResult result{start,std::vector<int>(n,distance_inf<int>()),std::vector<int>(n,-1)};
    std::vector<int> next(n+1),marked(n,-1),queue{start};
    std::iota(next.begin(),next.end(),1); next[n]=0;
    int prev=n;
    for(int v=0;v<n;v=next[prev]) { if(v==start) { next[prev]=next[v]; break; } prev=v; }
    result.dist[start]=0;
    for(std::size_t i=0;i<queue.size();++i) {
        int v=queue[i]; for(const auto& e:g[v]) marked[e.to]=v;
        prev=n;
        for(int u=next[prev];u<n;u=next[prev]) {
            if(marked[u]==v) { prev=u; continue; }
            result.dist[u]=result.dist[v]+1; result.parent[u]=v; queue.push_back(u); next[prev]=next[u];
        }
    }
    return result;
}
} // namespace cp
