#pragma once
#include <algorithm>
#include <cassert>
#include <optional>
#include <vector>
#include "potentialized_disjoint_set_union.hpp"
namespace cp {
// diff(u,v)=inverse(P[u])*P[v]。G: S, op, inverse, e。非可換群にも対応。
template<class G> class GroupPotentializedDisjointSetUnion {
public:
    using S=typename G::S;
    using MergeResult=PotentialMergeResult;
private:
    std::vector<int> parent_;
    std::vector<S> weight_;
    int components_;
    static int checked(int n) { assert(n>=0); return n; }
public:
    explicit GroupPotentializedDisjointSetUnion(int n=0) : parent_(checked(n),-1),weight_(n,G::e()),components_(n) {}
    int vertex_count() const { return int(parent_.size()); }
    int component_count() const { return components_; }
    int leader(int v) {
        assert(0<=v && v<vertex_count());
        if(parent_[v]<0) return v;
        int p=parent_[v],r=leader(p);
        weight_[v]=G::op(weight_[p],weight_[v]); parent_[v]=r; return r;
    }
    bool same(int u,int v) { return leader(u)==leader(v); }
    int size(int v) { return -parent_[leader(v)]; }
    S potential(int v) { leader(v); return weight_[v]; }
    std::optional<S> diff(int u,int v) {
        if(!same(u,v)) return std::nullopt;
        return G::op(G::inverse(weight_[u]),weight_[v]);
    }
    MergeResult merge(int u,int v,const S& difference) {
        int a=leader(u),b=leader(v);
        if(a==b) return G::op(G::inverse(weight_[u]),weight_[v])==difference?MergeResult::AlreadyConsistent:MergeResult::Contradiction;
        S shift=G::op(G::op(weight_[u],difference),G::inverse(weight_[v]));
        if(-parent_[a]<-parent_[b]) { std::swap(a,b); shift=G::inverse(shift); }
        parent_[a]+=parent_[b]; parent_[b]=a; weight_[b]=shift; --components_; return MergeResult::Merged;
    }
};
} // namespace cp
