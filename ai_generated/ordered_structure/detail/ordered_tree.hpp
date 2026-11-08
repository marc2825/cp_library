#pragma once
#include <cassert>
#include <cstdint>
#include <functional>
#include <optional>
#include <utility>
#include <vector>

namespace cp { namespace detail {
// TreapとRBSTの順位管理を共有。RBSTはノード数に比例したランダム結合。
template<class Key,class Compare,bool IsRBST> class OrderedTree {
    struct Node {
        std::optional<Key> key; std::uint64_t priority=0;
        int left=0,right=0,count=0,size=0,distinct=0;
    };
    std::vector<Node> nodes_{Node{}}; int root_=0; Compare comp_; std::uint64_t state_;
    std::uint64_t random() {
        state_+=0x9e3779b97f4a7c15ULL; auto x=state_;
        x=(x^(x>>30))*0xbf58476d1ce4e5b9ULL; x=(x^(x>>27))*0x94d049bb133111ebULL; return x^(x>>31);
    }
    int length(int k) const { return nodes_[k].size; }
    void pull(int k) {
        auto& a=nodes_[k]; a.size=a.count+length(a.left)+length(a.right);
        a.distinct=1+nodes_[a.left].distinct+nodes_[a.right].distinct;
    }
    std::pair<int,int> split(int k,const Key& x) {
        if(!k) return {0,0};
        if(comp_(*nodes_[k].key,x)) {
            auto [a,b]=split(nodes_[k].right,x); nodes_[k].right=a; pull(k); return {k,b};
        }
        auto [a,b]=split(nodes_[k].left,x); nodes_[k].left=b; pull(k); return {a,k};
    }
    int merge(int a,int b) {
        if(!a || !b) return a?a:b;
        bool left;
        if constexpr(IsRBST) left=random()%std::uint64_t(nodes_[a].distinct+nodes_[b].distinct)<std::uint64_t(nodes_[a].distinct);
        else left=nodes_[a].priority>nodes_[b].priority;
        if(left) { nodes_[a].right=merge(nodes_[a].right,b); pull(a); return a; }
        nodes_[b].left=merge(a,nodes_[b].left); pull(b); return b;
    }
    int insert_new(int k,int p) {
        if(!k) return p;
        bool top;
        if constexpr(IsRBST) top=random()%std::uint64_t(nodes_[k].distinct+1)==0;
        else top=nodes_[p].priority>nodes_[k].priority;
        if(top) { auto [a,b]=split(k,*nodes_[p].key); nodes_[p].left=a; nodes_[p].right=b; pull(p); return p; }
        if(comp_(*nodes_[p].key,*nodes_[k].key)) nodes_[k].left=insert_new(nodes_[k].left,p);
        else nodes_[k].right=insert_new(nodes_[k].right,p);
        pull(k); return k;
    }
    void increment(int k,const Key& x) {
        if(comp_(x,*nodes_[k].key)) increment(nodes_[k].left,x);
        else if(comp_(*nodes_[k].key,x)) increment(nodes_[k].right,x);
        else ++nodes_[k].count;
        pull(k);
    }
    int remove(int k,const Key& x) {
        if(comp_(x,*nodes_[k].key)) nodes_[k].left=remove(nodes_[k].left,x);
        else if(comp_(*nodes_[k].key,x)) nodes_[k].right=remove(nodes_[k].right,x);
        else if(nodes_[k].count>1) --nodes_[k].count;
        else return merge(nodes_[k].left,nodes_[k].right);
        pull(k); return k;
    }
public:
    explicit OrderedTree(Compare compare=Compare{},std::uint64_t seed=712367):comp_(compare),state_(seed) {}
    int size() const { return length(root_); }
    bool empty() const { return !root_; }
    void clear() { nodes_.assign(1,Node{}); root_=0; }
    int count(const Key& x) const {
        int k=root_;
        while(k) {
            if(comp_(x,*nodes_[k].key)) k=nodes_[k].left;
            else if(comp_(*nodes_[k].key,x)) k=nodes_[k].right;
            else return nodes_[k].count;
        }
        return 0;
    }
    void insert(const Key& x) {
        if(count(x)) increment(root_,x);
        else { int p=int(nodes_.size()); nodes_.push_back({x,random(),0,0,1,1,1}); root_=insert_new(root_,p); }
    }
    bool erase(const Key& x) { if(!count(x)) return false; root_=remove(root_,x); return true; }
    int order_of_key(const Key& x) const {
        int result=0,k=root_;
        while(k) {
            if(comp_(*nodes_[k].key,x)) { result+=length(nodes_[k].left)+nodes_[k].count; k=nodes_[k].right; }
            else k=nodes_[k].left;
        }
        return result;
    }
    std::optional<Key> kth(int p) const {
        if(p<0 || p>=size()) return std::nullopt;
        int k=root_;
        for (;;) {
            int l=length(nodes_[k].left);
            if(p<l) k=nodes_[k].left;
            else if(p<l+nodes_[k].count) return nodes_[k].key;
            else { p-=l+nodes_[k].count; k=nodes_[k].right; }
        }
    }
    std::optional<Key> lower_bound(const Key& x) const { return kth(order_of_key(x)); }
};
}} // namespace cp::detail
