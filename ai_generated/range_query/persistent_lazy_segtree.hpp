#pragma once
// 説明: docs/range_query/persistent_lazy_segtree.md
#include <cassert>
#include <cstddef>
#include <vector>

namespace cp {
// rootは同じインスタンス内だけで使用。mappingは長さを含むSに作用。
template<class S,auto op,auto e,class F,auto mapping,auto composition,auto id>
class PersistentLazySegtree {
    struct Node { S value; F lazy; int left=0,right=0; };
    int n_,initial_; std::vector<Node> nodes_;
    int create(Node x) { nodes_.push_back(x); return int(nodes_.size())-1; }
    int build(const std::vector<S>& a,int l,int r) {
        if(l==r) return create({e(),id()});
        if(r-l==1) return create({a[l],id()});
        int m=l+(r-l)/2,x=build(a,l,m),y=build(a,m,r);
        return create({op(nodes_[x].value,nodes_[y].value),id(),x,y});
    }
    int tagged(int k,F f) {
        Node n=nodes_[k]; n.value=mapping(f,n.value); n.lazy=composition(f,n.lazy); return create(n);
    }
    int update(int k,int l,int r,int a,int b,F f) {
        if(b<=l || r<=a) return k;
        if(a<=l && r<=b) return tagged(k,f);
        Node n=nodes_[k]; int m=l+(r-l)/2;
        int x=tagged(n.left,n.lazy),y=tagged(n.right,n.lazy);
        n.left=update(x,l,m,a,b,f); n.right=update(y,m,r,a,b,f); n.lazy=id();
        n.value=op(nodes_[n.left].value,nodes_[n.right].value); return create(n);
    }
    S query(int k,int l,int r,int a,int b,F carry) const {
        if(b<=l || r<=a) return e();
        const Node& n=nodes_[k];
        if(a<=l && r<=b) return mapping(carry,n.value);
        carry=composition(carry,n.lazy); int m=l+(r-l)/2;
        return op(query(n.left,l,m,a,b,carry),query(n.right,m,r,a,b,carry));
    }
    void bounds(int root,int l,int r) const {
        assert(0<=root && root<int(nodes_.size()) && 0<=l && l<=r && r<=n_);
        (void)root; (void)l; (void)r;
    }
public:
    explicit PersistentLazySegtree(const std::vector<S>& a={}) : n_(int(a.size())),initial_(0) { initial_=build(a,0,n_); }
    int initial_root() const { return initial_; }
    int size() const { return n_; }
    std::size_t node_count() const { return nodes_.size(); }
    int apply(int root,int l,int r,F f) { bounds(root,l,r); return l==r?root:update(root,0,n_,l,r,f); }
    S prod(int root,int l,int r) const { bounds(root,l,r); return l==r?e():query(root,0,n_,l,r,id()); }
    S get(int root,int p) const { return prod(root,p,p+1); }
};
} // namespace cp
