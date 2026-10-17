#pragma once
#include <array>
#include <numeric>
#include <vector>
#include "disjoint_set_union.hpp"

namespace cp {
// 葉0..n-1。成功した併合ごとに新しい内部頂点を一つ作る。
class MergeTree {
    DisjointSetUnion dsu_; std::vector<int> top_;
public:
    std::vector<int> parent;
    std::vector<std::array<int,2>> children;
    explicit MergeTree(int n):dsu_(n),top_(n),parent(n,-1),children(n,{-1,-1}) {
        std::iota(top_.begin(),top_.end(),0);
    }
    int leader(int v) { return dsu_.leader(v); }
    bool same(int u,int v) { return dsu_.same(u,v); }
    int component_root(int v) { return top_[leader(v)]; }
    // 同成分なら既存の根を返す。別成分なら作った内部頂点ID。
    int merge(int u,int v) {
        u=leader(u); v=leader(v); if (u==v) return top_[u];
        int a=top_[u],b=top_[v],k=int(parent.size());
        parent[a]=parent[b]=k; parent.push_back(-1); children.push_back({a,b});
        dsu_.merge(u,v); top_[leader(u)]=k; return k;
    }
    std::vector<int> roots() const {
        std::vector<int> r;
        for (int i=0;i<int(parent.size());++i) if (parent[i]==-1) r.push_back(i);
        return r;
    }
};
} // namespace cp
