#pragma once
#include <cassert>
#include <cstddef>
#include <vector>

namespace cp {
// default_prod(l,r)は未更新区間の積。空区間はe()。
// mapping/compositionはLazySegtreeと同じ。prodも伝播でノードを作る。
template<class S,auto op,auto e,class F,auto mapping,auto composition,auto id,auto default_prod>
class DynamicLazySegtree {
    struct Node { S value; F lazy=id(); int left=-1,right=-1; bool pending=false; };
    long long n_; std::vector<Node> nodes_;
    int make(long long l,long long r) {
        int k=int(nodes_.size()); nodes_.push_back({default_prod(l,r)}); return k;
    }
    void apply_node(int k,F f) {
        nodes_[k].value=mapping(f,nodes_[k].value);
        nodes_[k].lazy=composition(f,nodes_[k].lazy); nodes_[k].pending=true;
    }
    void push(int k,long long l,long long r) {
        auto m=l+(r-l)/2;
        if (nodes_[k].left==-1) {
            int a=make(l,m),b=make(m,r); nodes_[k].left=a; nodes_[k].right=b;
        }
        if (nodes_[k].pending) {
            F f=nodes_[k].lazy;
            apply_node(nodes_[k].left,f); apply_node(nodes_[k].right,f);
            nodes_[k].lazy=id(); nodes_[k].pending=false;
        }
    }
    void update(int k,long long l,long long r,long long a,long long b,F f) {
        if (b<=l || r<=a) return;
        if (a<=l && r<=b) { apply_node(k,f); return; }
        push(k,l,r); auto m=l+(r-l)/2;
        update(nodes_[k].left,l,m,a,b,f); update(nodes_[k].right,m,r,a,b,f);
        nodes_[k].value=op(nodes_[nodes_[k].left].value,nodes_[nodes_[k].right].value);
    }
    S query(int k,long long l,long long r,long long a,long long b) {
        if (b<=l || r<=a) return e();
        if (a<=l && r<=b) return nodes_[k].value;
        push(k,l,r); auto m=l+(r-l)/2;
        return op(query(nodes_[k].left,l,m,a,b),query(nodes_[k].right,m,r,a,b));
    }
public:
    explicit DynamicLazySegtree(long long n):n_(n) { assert(n>=0); make(0,n); }
    long long size() const { return n_; }
    std::size_t node_count() const { return nodes_.size(); }
    void apply(long long l,long long r,F f) {
        assert(0<=l && l<=r && r<=n_); if (l<r) update(0,0,n_,l,r,f);
    }
    S prod(long long l,long long r) {
        assert(0<=l && l<=r && r<=n_); return l==r?e():query(0,0,n_,l,r);
    }
    S get(long long p) { assert(0<=p && p<n_); return prod(p,p+1); }
    S all_prod() const { return nodes_[0].value; }
};
} // namespace cp
