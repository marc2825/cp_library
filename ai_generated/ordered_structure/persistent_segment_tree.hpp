#pragma once
#include <cassert>
#include <cstddef>
#include <vector>

namespace cp {
// 経路コピーによる永続モノイド木。rootはこのオブジェクト専用のハンドル。
// root=0は全要素e()。setは新rootを返す。docs/ordered_structure/persistent_segment_tree.md
template<class S, auto op, auto e> class PersistentSegmentTree {
    struct Node { S value=e(); int left=0,right=0; };
    int n_; std::vector<Node> nodes_{Node{}};
    int update(int root,int l,int r,int p,const S& x) {
        int k=int(nodes_.size()); nodes_.push_back(nodes_[root]);
        if(r-l==1) { nodes_[k].value=x; return k; }
        int m=l+(r-l)/2;
        if(p<m) { int child=update(nodes_[k].left,l,m,p,x); nodes_[k].left=child; }
        else { int child=update(nodes_[k].right,m,r,p,x); nodes_[k].right=child; }
        nodes_[k].value=op(nodes_[nodes_[k].left].value,nodes_[nodes_[k].right].value); return k;
    }
    S query(int k,int l,int r,int ql,int qr) const {
        if(!k || qr<=l || r<=ql) return e();
        if(ql<=l && r<=qr) return nodes_[k].value;
        int m=l+(r-l)/2; return op(query(nodes_[k].left,l,m,ql,qr),query(nodes_[k].right,m,r,ql,qr));
    }
    void root_check(int root) const { assert(0<=root && root<int(nodes_.size())); (void)root; }
public:
    explicit PersistentSegmentTree(int n):n_(n) { assert(n>=0); }
    int size() const { return n_; }
    std::size_t node_count() const { return nodes_.size(); }
    int empty_root() const { return 0; }
    int set(int root,int p,const S& x) { root_check(root); assert(0<=p && p<n_); return update(root,0,n_,p,x); }
    S prod(int root,int l,int r) const { root_check(root); assert(0<=l && l<=r && r<=n_); return query(root,0,n_,l,r); }
    S get(int root,int p) const { assert(0<=p && p<n_); return prod(root,p,p+1); }
};
} // namespace cp
