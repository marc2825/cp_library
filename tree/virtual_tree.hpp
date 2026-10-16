#pragma once
#include <algorithm>
#include <cassert>
#include <utility>
#include <vector>
#include "doubling_lca.hpp"

namespace cp {
struct VirtualTreeResult {
    std::vector<int> vertices; // local index -> 元の頂点。DFS順。
    std::vector<int> parent,roots,edge_length; // parent/rootはlocal index、根の辺長は0。
    std::vector<bool> marked;
};
// 前計算 O(n log n)、重要頂点k個の木を O(k log k+k log n) で構築。森・重複可。
template<class T=long long> class VirtualTree {
public:
    DoublingLCA<T> lca;
    explicit VirtualTree(const Graph<T>& g,int root=0):lca(g,root){}
    VirtualTreeResult build(std::vector<int> important) const {
        for(int v:important) { assert(0<=v && v<lca.tree.size()); (void)v; }
        auto less=[&](int a,int b){return lca.tree.tin[a]<lca.tree.tin[b];};
        std::sort(important.begin(),important.end(),less); important.erase(std::unique(important.begin(),important.end()),important.end());
        VirtualTreeResult out; out.vertices=important;
        for(std::size_t i=1;i<important.size();++i) { int v=lca.lca(important[i-1],important[i]); if(v!=-1) out.vertices.push_back(v); }
        std::sort(out.vertices.begin(),out.vertices.end(),less); out.vertices.erase(std::unique(out.vertices.begin(),out.vertices.end()),out.vertices.end());
        int m=int(out.vertices.size()); out.parent.assign(m,-1); out.edge_length.assign(m,0); out.marked.assign(m,false);
        std::vector<int> stack; std::size_t p=0;
        for(int i=0;i<m;++i) {
            int v=out.vertices[i];
            if(p<important.size() && important[p]==v) { out.marked[i]=true; ++p; }
            while(!stack.empty() && !lca.tree.is_ancestor(out.vertices[stack.back()],v)) stack.pop_back();
            if(stack.empty()) out.roots.push_back(i);
            else { out.parent[i]=stack.back(); out.edge_length[i]=lca.tree.depth[v]-lca.tree.depth[out.vertices[stack.back()]]; }
            stack.push_back(i);
        }
        return out;
    }
};
} // namespace cp
