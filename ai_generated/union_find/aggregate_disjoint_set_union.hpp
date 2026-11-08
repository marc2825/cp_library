#pragma once
#include <cassert>
#include <vector>
#include "disjoint_set_union.hpp"

namespace cp {
// 可換モノイド。addは頂点所属成分に寄与を追加、削除・上書きは扱わない。
template<class S,auto op,auto e> class AggregateDisjointSetUnion {
    DisjointSetUnion dsu_; std::vector<S> values_;
public:
    explicit AggregateDisjointSetUnion(int n):dsu_(n),values_(n,e()) {}
    explicit AggregateDisjointSetUnion(const std::vector<S>& a):dsu_(int(a.size())),values_(a) {}
    int leader(int v) { return dsu_.leader(v); }
    int size(int v) { return dsu_.size(v); }
    int component_count() const { return dsu_.component_count(); }
    bool same(int u,int v) { return dsu_.same(u,v); }
    S prod(int v) { return values_[leader(v)]; }
    void add(int v,const S& x) { v=leader(v); values_[v]=op(values_[v],x); }
    bool merge(int u,int v) {
        u=leader(u); v=leader(v); if (u==v) return false;
        S x=op(values_[u],values_[v]); dsu_.merge(u,v); values_[leader(u)]=x;
        return true;
    }
};
} // namespace cp
