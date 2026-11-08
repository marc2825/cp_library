#pragma once
// 説明: docs/graph_connectivity/tree_decomposition.md
#include <algorithm>
#include <cstdint>
#include <numeric>
#include <stdexcept>
#include <vector>
#include "graph.hpp"
namespace cp {
struct TreeDecomposition {
    int width=-1; bool optimal=false;
    std::vector<int> elimination_order;
    std::vector<std::vector<int>> bags; // bag ID = original vertex
    Graph<int> forest;
};
namespace tree_decomposition_detail {
template<class T> std::vector<std::vector<bool>> adjacency(const Graph<T>& g) {
    if(g.directed()) throw std::invalid_argument("undirected decomposition required");
    std::vector<std::vector<bool>> a(g.size(),std::vector<bool>(g.size()));
    for(auto e:g.edges()) if(e.from!=e.to) a[e.from][e.to]=a[e.to][e.from]=true;
    return a;
}
inline TreeDecomposition build(std::vector<std::vector<bool>> a,const std::vector<int>& order,bool optimal) {
    int n=int(a.size()); TreeDecomposition r; r.optimal=optimal; r.elimination_order=order; r.bags.resize(n); r.forest=Graph<int>(n);
    std::vector<bool> alive(n,true); std::vector<int> position(n); for(int i=0;i<n;++i) position[order[i]]=i;
    for(int v:order) {
        auto& bag=r.bags[v]; bag.push_back(v); int parent=-1;
        for(int u=0;u<n;++u) if(alive[u] && a[v][u]) { bag.push_back(u); if(parent<0 || position[u]<position[parent]) parent=u; }
        r.width=std::max(r.width,int(bag.size())-1); if(parent>=0) r.forest.add_edge(v,parent);
        for(int u:bag) for(int w:bag) if(u!=w) a[u][w]=true;
        alive[v]=false;
    }
    return r;
}
} // namespace tree_decomposition_detail
// 厳密な最小木幅。n<=22、O(n^2 2^n) 時間、O(2^n+n^2) 領域。
template<class T> TreeDecomposition minimum_tree_decomposition(const Graph<T>& g) {
    auto a=tree_decomposition_detail::adjacency(g); int n=g.size();
    if(n>22) throw std::length_error("exact treewidth requires n <= 22");
    using Mask=std::uint32_t; Mask size=Mask(1)<<n,full=size-1;
    std::vector<Mask> neighbors(n); for(int v=0;v<n;++v) for(int u=0;u<n;++u) if(a[v][u]) neighbors[v]|=Mask(1)<<u;
    std::vector<unsigned char> dp(size,255),last(size,255); dp[0]=0;
    for(Mask s=0;s<size;++s) for(int v=0;v<n;++v) if(!(s>>v&1)) {
        Mask reached=Mask(1)<<v,front=neighbors[v]&s,boundary=neighbors[v]&~s;
        while(front) {
            int u=__builtin_ctz(front); front&=front-1;
            if(reached>>u&1) continue;
            reached|=Mask(1)<<u; boundary|=neighbors[u]&~s; front|=neighbors[u]&s&~reached;
        }
        boundary&=full^(Mask(1)<<v); int width=std::max(int(dp[s]),__builtin_popcount(boundary)); Mask next=s|(Mask(1)<<v);
        if(width<dp[next]) { dp[next]=static_cast<unsigned char>(width); last[next]=static_cast<unsigned char>(v); }
    }
    std::vector<int> order;
    for(Mask s=full;s;) { int v=last[s]; order.push_back(v); s^=Mask(1)<<v; }
    std::reverse(order.begin(),order.end()); return tree_decomposition_detail::build(std::move(a),order,true);
}
// Min-fill heuristic。返り値は木幅の上界、最適性保証はない。O(n^4) 時間。
template<class T> TreeDecomposition greedy_tree_decomposition(const Graph<T>& g) {
    auto original=tree_decomposition_detail::adjacency(g),a=original; int n=g.size();
    std::vector<bool> alive(n,true); std::vector<int> order;
    for(int step=0;step<n;++step) {
        int best=-1,best_fill=0,best_degree=0;
        for(int v=0;v<n;++v) if(alive[v]) {
            int degree=0,fill=0;
            for(int u=0;u<n;++u) if(alive[u] && a[v][u]) { ++degree; for(int w=0;w<u;++w) if(alive[w] && a[v][w] && !a[u][w]) ++fill; }
            if(best<0 || fill<best_fill || (fill==best_fill && degree<best_degree)) { best=v; best_fill=fill; best_degree=degree; }
        }
        for(int u=0;u<n;++u) if(alive[u] && a[best][u]) for(int w=0;w<n;++w) if(alive[w] && a[best][w] && u!=w) a[u][w]=true;
        alive[best]=false; order.push_back(best);
    }
    return tree_decomposition_detail::build(std::move(original),order,false);
}
} // namespace cp
