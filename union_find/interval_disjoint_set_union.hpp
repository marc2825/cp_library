#pragma once
#include <cassert>
#include <numeric>
#include <vector>
#include "disjoint_set_union.hpp"

namespace cp {
// [l,r)内の全頂点を連結。処理済みの隣接境界を飛ばす。
class IntervalDisjointSetUnion {
    DisjointSetUnion dsu_; std::vector<int> next_;
    int next(int x) {
        int r=x;
        while (next_[r]!=r) r=next_[r];
        while (next_[x]!=x) { int y=next_[x]; next_[x]=r; x=y; }
        return r;
    }
public:
    explicit IntervalDisjointSetUnion(int n):dsu_(n),next_(n+1) { std::iota(next_.begin(),next_.end(),0); }
    int leader(int v) { return dsu_.leader(v); }
    int size(int v) { return dsu_.size(v); }
    bool same(int u,int v) { return dsu_.same(u,v); }
    int component_count() const { return dsu_.component_count(); }
    int merge(int l,int r) {
        assert(0<=l && l<=r && r<=dsu_.vertex_count());
        int count=0;
        for (int i=next(l);i+1<r;i=next(i)) {
            count+=dsu_.merge(i,i+1); next_[i]=next(i+1);
        }
        return count;
    }
};
} // namespace cp
