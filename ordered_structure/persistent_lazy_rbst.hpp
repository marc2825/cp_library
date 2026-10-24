#pragma once
#include <cassert>
#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

namespace cp {
// コピーで版を分岐できる動的列。サイズ比で根を選ぶRBST、経路コピー、遅延作用。
// mappingはopを保つ一様な作用、composition(f,g)はf after g。
template<class S, auto op, auto e, class F, auto mapping, auto composition, auto id>
class PersistentLazyRBST {
    struct Node;
    using Ptr=std::shared_ptr<const Node>;
    struct Node {
        S value,forward,backward; Ptr left,right; int size=1;
        F lazy=id(); bool pending=false,reverse=false;
        explicit Node(const S& x):value(x),forward(x),backward(x) {}
    };
    Ptr root_; std::uint64_t state_;
    static int length(const Ptr& p) { return p?p->size:0; }
    static S product(const Ptr& p,bool reverse=false) { return p?(reverse?p->backward:p->forward):e(); }
    static void pull(Node& p) {
        p.size=length(p.left)+1+length(p.right);
        p.forward=op(op(product(p.left),p.value),product(p.right));
        p.backward=op(op(product(p.right,true),p.value),product(p.left,true));
    }
    static Ptr acted(const Ptr& p,const F& f) {
        if(!p) return p;
        auto q=std::make_shared<Node>(*p);
        q->value=mapping(f,q->value); q->forward=mapping(f,q->forward); q->backward=mapping(f,q->backward);
        q->lazy=q->pending?composition(f,q->lazy):f; q->pending=true; return q;
    }
    static Ptr flipped(const Ptr& p) {
        if(!p) return p;
        auto q=std::make_shared<Node>(*p);
        std::swap(q->left,q->right); std::swap(q->forward,q->backward); q->reverse=!q->reverse; return q;
    }
    static std::shared_ptr<Node> pushed(const Ptr& p) {
        auto q=std::make_shared<Node>(*p);
        if(q->pending) { q->left=acted(q->left,q->lazy); q->right=acted(q->right,q->lazy); q->lazy=id(); q->pending=false; }
        if(q->reverse) { q->left=flipped(q->left); q->right=flipped(q->right); q->reverse=false; }
        return q;
    }
    std::uint64_t random() {
        state_+=0x9e3779b97f4a7c15ULL; auto x=state_;
        x=(x^(x>>30))*0xbf58476d1ce4e5b9ULL; x=(x^(x>>27))*0x94d049bb133111ebULL; return x^(x>>31);
    }
    std::uint64_t bounded(std::uint64_t n) {
        auto threshold=(std::uint64_t(0)-n)%n, x=random();
        while(x<threshold) x=random();
        return x%n;
    }
    Ptr join(const Ptr& a,const Ptr& b) {
        if(!a || !b) return a?a:b;
        if(bounded(std::uint64_t(length(a))+length(b))<std::uint64_t(length(a))) {
            auto q=pushed(a); q->right=join(q->right,b); pull(*q); return q;
        }
        auto q=pushed(b); q->left=join(a,q->left); pull(*q); return q;
    }
    static std::pair<Ptr,Ptr> cut(const Ptr& p,int count) {
        if(!p) return {};
        if(!count) return {{},p};
        if(count==length(p)) return {p,{}};
        auto q=pushed(p); int n=length(q->left);
        if(count<=n) { auto [a,b]=cut(q->left,count); q->left=b; pull(*q); return {a,q}; }
        auto [a,b]=cut(q->right,count-n-1); q->right=a; pull(*q); return {q,b};
    }
    static S query(const Ptr& p,int l,int r) {
        if(l==r) return e();
        if(!l && r==length(p)) return p->forward;
        auto q=pushed(p); int n=length(q->left); S result=e();
        if(l<n) result=query(q->left,l,r<n?r:n);
        if(l<=n && n<r) result=op(result,q->value);
        if(n+1<r) result=op(result,query(q->right,l>n+1?l-n-1:0,r-n-1));
        return result;
    }
    static void collect(const Ptr& p,std::vector<S>& out) {
        if(!p) return;
        auto q=pushed(p); collect(q->left,out); out.push_back(q->value); collect(q->right,out);
    }
    void bounds(int l,int r) const { assert(0<=l && l<=r && r<=size()); (void)l; (void)r; }
public:
    explicit PersistentLazyRBST(std::uint64_t seed=0x51f15eULL):state_(seed) {}
    explicit PersistentLazyRBST(const std::vector<S>& values,std::uint64_t seed=0x51f15eULL):state_(seed) {
        for(const auto& x:values) root_=join(root_,std::make_shared<Node>(x));
    }
    int size() const { return length(root_); }
    bool empty() const { return !root_; }
    void clear() { root_.reset(); }
    S all_prod() const { return product(root_); }
    S prod(int l,int r) const { bounds(l,r); return query(root_,l,r); }
    S get(int p) const { assert(0<=p && p<size()); return prod(p,p+1); }
    void insert(int p,const S& x) {
        bounds(p,p); auto [a,b]=cut(root_,p); root_=join(join(a,std::make_shared<Node>(x)),b);
    }
    void erase(int p) {
        assert(0<=p && p<size()); auto [a,b]=cut(root_,p); auto [mid,c]=cut(b,1); (void)mid; root_=join(a,c);
    }
    void set(int p,const S& x) {
        assert(0<=p && p<size()); auto [a,b]=cut(root_,p); auto [mid,c]=cut(b,1); (void)mid;
        root_=join(join(a,std::make_shared<Node>(x)),c);
    }
    void apply(int l,int r,const F& f) {
        bounds(l,r); auto [a,b]=cut(root_,l); auto [mid,c]=cut(b,r-l); root_=join(join(a,acted(mid,f)),c);
    }
    void reverse(int l,int r) {
        bounds(l,r); auto [a,b]=cut(root_,l); auto [mid,c]=cut(b,r-l); root_=join(join(a,flipped(mid)),c);
    }
    void move(int l,int r,int p) {
        bounds(l,r); assert(0<=p && p<=size()-(r-l));
        auto [a,b]=cut(root_,l); auto [mid,c]=cut(b,r-l); auto [x,y]=cut(join(a,c),p); root_=join(join(x,mid),y);
    }
    std::pair<PersistentLazyRBST,PersistentLazyRBST> split(int p) const {
        bounds(p,p); auto [a,b]=cut(root_,p); auto x=*this,y=*this; x.root_=a; y.root_=b; return {x,y};
    }
    PersistentLazyRBST concat(const PersistentLazyRBST& other) const {
        auto result=*this; result.root_=result.join(root_,other.root_); return result;
    }
    std::vector<S> to_vector() const { std::vector<S> out; out.reserve(size()); collect(root_,out); return out; }
};
} // namespace cp
