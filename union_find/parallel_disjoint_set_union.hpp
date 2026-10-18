#pragma once
// 説明: docs/union_find/parallel_disjoint_set_union.md
#include <cassert>
#include <vector>
#include "disjoint_set_union.hpp"

namespace cp {
// 区間[l,l+len),[r,r+len)を同じoffset同士で併合するParallel Union-Find。
class ParallelDisjointSetUnion {
    int n_; std::vector<DisjointSetUnion> levels_;
    void unite(int k,int a,int b) {
        if (!levels_[k].merge(a,b) || !k) return;
        unite(k-1,a,b); unite(k-1,a+(1<<(k-1)),b+(1<<(k-1)));
    }
public:
    explicit ParallelDisjointSetUnion(int n) : n_(n) {
        assert(n>=0); levels_.emplace_back(n);
        for (int k=1;k<31 && (1LL<<k)<=n;++k) levels_.emplace_back(n-(1<<k)+1);
    }
    void merge_ranges(int a,int b,int length) {
        assert(0<=a && 0<=b && 0<=length && length<=n_-a && length<=n_-b);
        if (!length) return;
        int k=0; while ((1LL<<(k+1))<=length) ++k;
        unite(k,a,b); unite(k,a+length-(1<<k),b+length-(1<<k));
    }
    bool merge(int a,int b) { return levels_[0].merge(a,b); }
    int leader(int v) { return levels_[0].leader(v); }
    bool same(int a,int b) { return levels_[0].same(a,b); }
    int size(int v) { return levels_[0].size(v); }
    int component_count() const { return levels_[0].component_count(); }
};
} // namespace cp
