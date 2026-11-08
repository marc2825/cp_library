#pragma once
// 説明: docs/ordered_structure/splay_tree.md
#include <cassert>
#include <functional>
#include <optional>
#include <vector>

namespace cp {
// key多重集合。参照した節点/最後の検索節点をsplay。操作は償却O(log n)。
template<class Key,class Compare=std::less<Key>> class SplayTree {
    struct Node { Key key; int parent=0,left=0,right=0,count=1,size=1; };
    std::vector<Node> nodes_; int root_=0; Compare comp_;
    int length(int k) const { return k?nodes_[k-1].size:0; }
    Node& at(int k) { return nodes_[k-1]; }
    const Node& at(int k) const { return nodes_[k-1]; }
    void pull(int k) { at(k).size=length(at(k).left)+length(at(k).right)+at(k).count; }
    void rotate(int x) {
        int p=at(x).parent,g=at(p).parent; bool right=at(p).right==x; int middle=right?at(x).left:at(x).right;
        if(g) { if(at(g).left==p) at(g).left=x; else at(g).right=x; } else root_=x;
        at(x).parent=g;
        if(right) { at(p).right=middle; at(x).left=p; } else { at(p).left=middle; at(x).right=p; }
        at(p).parent=x; if(middle) at(middle).parent=p; pull(p); pull(x);
    }
    void splay(int x) {
        while(at(x).parent) {
            int p=at(x).parent,g=at(p).parent;
            if(g) rotate((at(g).left==p)==(at(p).left==x)?p:x);
            rotate(x);
        }
    }
    int find(const Key& key) {
        int k=root_,last=0;
        while(k) { last=k; if(comp_(key,at(k).key)) k=at(k).left; else if(comp_(at(k).key,key)) k=at(k).right; else { splay(k); return k; } }
        if(last) splay(last);
        return 0;
    }
public:
    explicit SplayTree(Compare comp=Compare{}) : comp_(comp) {}
    int size() const { return length(root_); }
    bool empty() const { return !root_; }
    void clear() { nodes_.clear(); root_=0; }
    int count(const Key& key) { int k=find(key); return k?at(k).count:0; }
    void insert(const Key& key) {
        if(!root_) { nodes_.push_back({key}); root_=int(nodes_.size()); return; }
        int k=root_;
        for(;;) {
            if(!comp_(key,at(k).key) && !comp_(at(k).key,key)) { ++at(k).count; pull(k); splay(k); return; }
            bool right=comp_(at(k).key,key); int next=right?at(k).right:at(k).left;
            if(next) { k=next; continue; }
            nodes_.push_back({key,k}); int x=int(nodes_.size());
            if(right) at(k).right=x; else at(k).left=x;
            splay(x); return;
        }
    }
    bool erase(const Key& key) {
        int k=find(key); if(!k) return false;
        if(at(k).count>1) { --at(k).count; pull(k); return true; }
        int left=at(k).left,right=at(k).right;
        if(left) at(left).parent=0;
        if(right) at(right).parent=0;
        root_=left;
        if(!left) { root_=right; return true; }
        int x=left; while(at(x).right) x=at(x).right;
        splay(x); at(x).right=right; if(right) at(right).parent=x; pull(x); return true;
    }
    int order_of_key(const Key& key) {
        int k=root_,last=0,result=0;
        while(k) { last=k; if(comp_(at(k).key,key)) { result+=length(at(k).left)+at(k).count; k=at(k).right; } else k=at(k).left; }
        if(last) splay(last);
        return result;
    }
    std::optional<Key> kth(int rank) {
        if(rank<0 || rank>=size()) return std::nullopt;
        int k=root_;
        for(;;) { int l=length(at(k).left); if(rank<l) k=at(k).left; else if(rank<l+at(k).count) { splay(k); return at(k).key; } else { rank-=l+at(k).count; k=at(k).right; } }
    }
};
} // namespace cp
