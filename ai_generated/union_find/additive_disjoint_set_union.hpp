#pragma once
// 説明: docs/union_find/additive_disjoint_set_union.md
#include <cassert>
#include <utility>
#include <vector>

namespace cp {
// 成分加算・頂点加算・頂点値・成分和。merge前の値を保存するoffset。
template<class T=long long> class AdditiveDisjointSetUnion {
    std::vector<int> parent_; std::vector<T> offset_,base_,sum_; int components_;
    int root(int v) {
        if(parent_[v]<0) return v;
        int p=parent_[v],r=root(p);
        if(p!=r) offset_[v]=offset_[v]+offset_[p];
        return parent_[v]=r;
    }
    void check(int v) const { assert(0<=v && v<int(parent_.size())); (void)v; }
public:
    explicit AdditiveDisjointSetUnion(const std::vector<T>& values) : parent_(values.size(),-1),offset_(values.size(),T(0)),base_(values),sum_(values),components_(int(values.size())) {}
    explicit AdditiveDisjointSetUnion(int n=0) : AdditiveDisjointSetUnion(std::vector<T>(n,T(0))) {}
    int leader(int v) { check(v); return root(v); }
    bool same(int a,int b) { return leader(a)==leader(b); }
    int size(int v) { return -parent_[leader(v)]; }
    int component_count() const { return components_; }
    T get(int v) { int r=leader(v); return base_[v]+(v==r?offset_[r]:offset_[v]+offset_[r]); }
    T component_sum(int v) { return sum_[leader(v)]; }
    void add_component(int v,T x) { int r=leader(v); offset_[r]=offset_[r]+x; sum_[r]=sum_[r]+x*T(-parent_[r]); }
    void add_vertex(int v,T x) { int r=leader(v); base_[v]=base_[v]+x; sum_[r]=sum_[r]+x; }
    bool merge(int a,int b) {
        int u=leader(a),v=leader(b); if(u==v) return false;
        if(parent_[u]>parent_[v]) std::swap(u,v);
        offset_[v]=offset_[v]-offset_[u]; parent_[u]+=parent_[v]; parent_[v]=u; sum_[u]=sum_[u]+sum_[v]; --components_; return true;
    }
};
} // namespace cp
