#pragma once
#include <algorithm>
#include <cassert>
#include <cstdint>
#include <utility>
#include <vector>

namespace cp {
// 動的列のモノイド集約。反転用に順方向・逆方向の積を保持。期待O(log n)/操作。
// 削除ノードはclearまで保持。説明: docs/ordered_structure/implicit_treap.md
template<class S, auto op, auto e> class ImplicitTreap {
    struct Node { S value,forward,backward; std::uint64_t priority; int left=0,right=0,size=1; bool reverse=false; };
    std::vector<Node> nodes_; int root_=0; std::uint64_t state_;
    int length(int k) const { return k?nodes_[k].size:0; }
    S product(int k,bool reverse=false) const { return k?(reverse?nodes_[k].backward:nodes_[k].forward):e(); }
    std::uint64_t random() { state_+=0x9e3779b97f4a7c15ULL; auto x=state_; x=(x^(x>>30))*0xbf58476d1ce4e5b9ULL; x=(x^(x>>27))*0x94d049bb133111ebULL; return x^(x>>31); }
    void flip(int k) { if(k) { auto &n=nodes_[k]; std::swap(n.left,n.right); std::swap(n.forward,n.backward); n.reverse=!n.reverse; } }
    void push(int k) { if(nodes_[k].reverse) { flip(nodes_[k].left); flip(nodes_[k].right); nodes_[k].reverse=false; } }
    void pull(int k) { auto &n=nodes_[k]; n.size=1+length(n.left)+length(n.right); n.forward=op(op(product(n.left),n.value),product(n.right)); n.backward=op(op(product(n.right,true),n.value),product(n.left,true)); }
    std::pair<int,int> split(int k,int count) {
        if(!k) return {0,0};
        push(k); int left=length(nodes_[k].left);
        if(count<=left) { auto [a,b]=split(nodes_[k].left,count); nodes_[k].left=b; pull(k); return {a,k}; }
        auto [a,b]=split(nodes_[k].right,count-left-1); nodes_[k].right=a; pull(k); return {k,b};
    }
    int merge(int a,int b) {
        if(!a || !b) return a?a:b;
        if(nodes_[a].priority>nodes_[b].priority) { push(a); nodes_[a].right=merge(nodes_[a].right,b); pull(a); return a; }
        push(b); nodes_[b].left=merge(a,nodes_[b].left); pull(b); return b;
    }
    void bounds(int l,int r) const { assert(0<=l && l<=r && r<=size()); (void)l; (void)r; }
public:
    explicit ImplicitTreap(std::uint64_t seed=0x51f15eULL):state_(seed) { nodes_.push_back({e(),e(),e(),0,0,0,0,false}); }
    explicit ImplicitTreap(const std::vector<S>& a,std::uint64_t seed=0x51f15eULL):ImplicitTreap(seed) { for(const S& x:a) insert(size(),x); }
    int size() const { return length(root_); }
    bool empty() const { return !root_; }
    void clear() { nodes_.erase(nodes_.begin()+1,nodes_.end()); root_=0; }
    void insert(int p,const S& x) {
        assert(0<=p && p<=size()); int k=int(nodes_.size()); nodes_.push_back({x,x,x,random()});
        auto [a,b]=split(root_,p); root_=merge(merge(a,k),b);
    }
    void erase(int p) { assert(0<=p && p<size()); auto [a,b]=split(root_,p); auto [mid,c]=split(b,1); (void)mid; root_=merge(a,c); }
    void set(int p,const S& x) { assert(0<=p && p<size()); auto [a,b]=split(root_,p); auto [mid,c]=split(b,1); nodes_[mid].value=x; pull(mid); root_=merge(a,merge(mid,c)); }
    S get(int p) { assert(0<=p && p<size()); return prod(p,p+1); }
    S prod(int l,int r) { bounds(l,r); auto [a,b]=split(root_,l); auto [mid,c]=split(b,r-l); S result=product(mid); root_=merge(a,merge(mid,c)); return result; }
    S all_prod() const { return product(root_); }
    void reverse(int l,int r) { bounds(l,r); auto [a,b]=split(root_,l); auto [mid,c]=split(b,r-l); flip(mid); root_=merge(a,merge(mid,c)); }
    // [l,r)を取り出し、残った列のp番目の直前へ挿入。
    void move(int l,int r,int p) { bounds(l,r); assert(0<=p && p<=size()-(r-l)); auto [a,b]=split(root_,l); auto [mid,c]=split(b,r-l); auto [x,y]=split(merge(a,c),p); root_=merge(x,merge(mid,y)); }
    std::vector<S> to_vector() {
        std::vector<S> result; result.reserve(size()); std::vector<int> stack; int k=root_;
        while(k || !stack.empty()) { while(k) { push(k); stack.push_back(k); k=nodes_[k].left; } k=stack.back(); stack.pop_back(); result.push_back(nodes_[k].value); k=nodes_[k].right; }
        return result;
    }
};
} // namespace cp
