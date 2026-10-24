#pragma once
#include <cassert>
#include <cstddef>
#include <memory>
#include <stdexcept>
#include <utility>

namespace cp {
// 未作成点は e()。不変ノードの共有によりコピー O(1)、更新は経路コピー。
template<class S,auto op,auto e> class DynamicSegtree {
    struct Node;
    using Ptr=std::shared_ptr<const Node>;
    struct Node {
        S value;
        Ptr left,right;
        std::size_t count;
        Node(S v,Ptr a={},Ptr b={}) : value(std::move(v)),left(std::move(a)),right(std::move(b)),
            count(1+(left?left->count:0)+(right?right->count:0)) {}
    };
    long long n_;
    Ptr root_;
    static S value(const Ptr& p) { return p?p->value:e(); }
    static Ptr branch(Ptr a,Ptr b) {
        if (!a && !b) return {};
        S x=op(value(a),value(b));
        return std::make_shared<Node>(std::move(x),std::move(a),std::move(b));
    }
    static Ptr update(const Ptr& k,long long l,long long r,long long p,const S& x) {
        if (r-l==1) return std::make_shared<Node>(x);
        long long m=l+(r-l)/2;
        Ptr a=k?k->left:Ptr{},b=k?k->right:Ptr{};
        if (p<m) a=update(a,l,m,p,x); else b=update(b,m,r,p,x);
        return branch(std::move(a),std::move(b));
    }
    static S query(const Ptr& k,long long l,long long r,long long a,long long b) {
        if (!k || b<=l || r<=a) return e();
        if (a<=l && r<=b) return k->value;
        long long m=l+(r-l)/2;
        return op(query(k->left,l,m,a,b),query(k->right,m,r,a,b));
    }
    static std::pair<Ptr,Ptr> cut(const Ptr& k,long long l,long long r,long long p) {
        if (!k) return {};
        if (r<=p) return {k,{}};
        if (p<=l) return {{},k};
        long long m=l+(r-l)/2;
        if (p<m) { auto [a,b]=cut(k->left,l,m,p); return {branch(a,{}),branch(b,k->right)}; }
        auto [a,b]=cut(k->right,m,r,p); return {branch(k->left,a),branch({},b)};
    }
    template<class Combine> static Ptr join(const Ptr& a,const Ptr& b,long long l,long long r,Combine& combine) {
        if (!a || !b) return a?a:b;
        if (r-l==1) return std::make_shared<Node>(combine(a->value,b->value));
        long long m=l+(r-l)/2;
        auto left=join(a->left,b->left,l,m,combine);
        auto right=join(a->right,b->right,m,r,combine);
        return branch(std::move(left),std::move(right));
    }
public:
    explicit DynamicSegtree(long long n) : n_(n) { assert(n>=0); }
    long long size() const { return n_; }
    std::size_t node_count() const { return root_?root_->count:0; }
    void set(long long p,const S& x) { assert(0<=p && p<n_); root_=update(root_,0,n_,p,x); }
    S get(long long p) const { assert(0<=p && p<n_); return prod(p,p+1); }
    S prod(long long l,long long r) const { assert(0<=l && l<=r && r<=n_); return query(root_,0,n_,l,r); }
    S all_prod() const { return value(root_); }
    void clear() { root_.reset(); }
    // 両方とも元の座標域を保持する。右側の座標を 0 へ詰め直さない。
    std::pair<DynamicSegtree,DynamicSegtree> split_copy(long long p) const {
        assert(0<=p && p<=n_);
        auto [a,b]=cut(root_,0,n_,p);
        DynamicSegtree left(n_),right(n_); left.root_=std::move(a); right.root_=std::move(b);
        return {std::move(left),std::move(right)};
    }
    DynamicSegtree split(long long p) {
        auto parts=split_copy(p); root_=std::move(parts.first.root_); return std::move(parts.second);
    }
    template<class Combine> DynamicSegtree merged(const DynamicSegtree& other,Combine combine) const {
        if (n_!=other.n_) throw std::invalid_argument("DynamicSegtree universe mismatch");
        DynamicSegtree result(n_); result.root_=join(root_,other.root_,0,n_,combine); return result;
    }
    DynamicSegtree merged(const DynamicSegtree& other) const { return merged(other,op); }
    // other を空にする。重複点は combine(自分,other)。既存コピーは保持。
    template<class Combine> void merge(DynamicSegtree& other,Combine combine) {
        if (this==&other) throw std::invalid_argument("DynamicSegtree self consumption");
        auto result=merged(other,std::move(combine)); root_=std::move(result.root_); other.clear();
    }
    void merge(DynamicSegtree& other) { merge(other,op); }
};
} // namespace cp
