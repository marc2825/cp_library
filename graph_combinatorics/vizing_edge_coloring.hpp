#pragma once
// Adapted from KACTL EdgeColoring.h, Simon Lindholm, CC0.
// Pinned source and license: docs/graph_extensions/references.md.
#include <algorithm>
#include <set>
#include <stdexcept>
#include <utility>
#include <vector>
#include "../graph/graph.hpp"

namespace cp {
struct VizingEdgeColoring { int max_degree=0,color_count=0; std::vector<int> color; };
// Simple undirected graph. Misra-Gries fan rotations: O(VE), O(V(Delta+1)+E).
template<class T> VizingEdgeColoring vizing_edge_coloring(const Graph<T>& g) {
    if (g.directed()) throw std::invalid_argument("edge coloring requires undirected graph");
    int n=g.size(); std::vector<int> degree(n),fan(n),free(n),fan_color(n+1);
    std::set<std::pair<int,int>> seen;
    for (auto e:g.edges()) {
        if (e.from==e.to || !seen.insert(std::minmax(e.from,e.to)).second)
            throw std::invalid_argument("Vizing requires a simple graph");
        ++degree[e.from]; ++degree[e.to];
    }
    VizingEdgeColoring r; r.color.resize(g.edge_count());
    for (int d:degree) r.max_degree=std::max(r.max_degree,d);
    int colors=r.max_degree+1;
    std::vector<std::vector<int>> adj(n,std::vector<int>(colors,-1));
    for (auto edge:g.edges()) {
        int u=edge.from,v=edge.to; fan[0]=v;
        std::vector<int> location(colors);
        int at=u,end=u,d,c=free[u],length=0,i=0;
        while (true) {
            d=free[v];
            if (location[d] || adj[u][d]<0) break;
            v=adj[u][d]; location[d]=++length; fan_color[length]=d; fan[length]=v;
        }
        fan_color[location[d]]=c;
        for (int cd=d; at>=0; cd^=c^d,at=adj[at][cd]) {
            end=at; std::swap(adj[at][cd],adj[at][cd^c^d]);
        }
        while (adj[fan[i]][d]>=0) {
            int left=fan[i],right=fan[i+1],color=fan_color[++i];
            adj[u][color]=left; adj[left][color]=u; adj[right][color]=-1; free[right]=color;
        }
        adj[u][d]=fan[i]; adj[fan[i]][d]=u;
        for (int vtx:{fan[0],u,end}) {free[vtx]=0; while (adj[vtx][free[vtx]]>=0) ++free[vtx];}
    }
    for (auto e:g.edges()) {
        int c=0; while (adj[e.from][c]!=e.to) ++c;
        r.color[e.idx]=c; r.color_count=std::max(r.color_count,c+1);
    }
    return r;
}
} // namespace cp
