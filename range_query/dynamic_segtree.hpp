#pragma once
#include <cassert>
#include <cstddef>
#include <vector>

namespace cp {
// 未作成点はe()。長さはlong long、ノードは更新した経路のみ確保。
template<class S,auto op,auto e> class DynamicSegtree {
    struct Node { S value=e(); int left=0,right=0; };
    long long n_; int root_=0; std::vector<Node> nodes_{Node{}};
    int update(int k,long long l,long long r,long long p,const S& x) {
        if (!k) { k=int(nodes_.size()); nodes_.push_back(Node{}); }
        if (r-l==1) { nodes_[k].value=x; return k; }
        auto m=l+(r-l)/2;
        if (p<m) { int c=update(nodes_[k].left,l,m,p,x); nodes_[k].left=c; }
        else { int c=update(nodes_[k].right,m,r,p,x); nodes_[k].right=c; }
        nodes_[k].value=op(nodes_[nodes_[k].left].value,nodes_[nodes_[k].right].value);
        return k;
    }
    S query(int k,long long l,long long r,long long a,long long b) const {
        if (!k || b<=l || r<=a) return e();
        if (a<=l && r<=b) return nodes_[k].value;
        auto m=l+(r-l)/2;
        return op(query(nodes_[k].left,l,m,a,b),query(nodes_[k].right,m,r,a,b));
    }
public:
    explicit DynamicSegtree(long long n):n_(n) { assert(n>=0); }
    long long size() const { return n_; }
    std::size_t node_count() const { return nodes_.size()-1; }
    void set(long long p,const S& x) { assert(0<=p && p<n_); root_=update(root_,0,n_,p,x); }
    S get(long long p) const { assert(0<=p && p<n_); return prod(p,p+1); }
    S prod(long long l,long long r) const { assert(0<=l && l<=r && r<=n_); return query(root_,0,n_,l,r); }
    S all_prod() const { return nodes_[root_].value; }
};
} // namespace cp
