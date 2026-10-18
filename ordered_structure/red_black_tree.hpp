#pragma once
// 説明: docs/ordered_structure/red_black_tree.md
#include <algorithm>
#include <functional>
#include <optional>
#include <vector>

namespace cp {
// 左傾Red-Black Tree。key多重集合、最悪O(log n)。添字所有で破棄は非再帰。
template<class Key,class Compare=std::less<Key>> class RedBlackTree {
    struct Node { Key key; int left=0,right=0,count=1,size=1; bool red=true; };
    std::vector<Node> nodes_; int root_=0; Compare comp_;
    Node& at(int k) { return nodes_[k-1]; }
    const Node& at(int k) const { return nodes_[k-1]; }
    bool red(int k) const { return k && at(k).red; }
    int length(int k) const { return k?at(k).size:0; }
    void pull(int k) { at(k).size=length(at(k).left)+at(k).count+length(at(k).right); }
    int left_rotate(int k) { int x=at(k).right; at(k).right=at(x).left; at(x).left=k; at(x).red=at(k).red; at(k).red=true; pull(k); pull(x); return x; }
    int right_rotate(int k) { int x=at(k).left; at(k).left=at(x).right; at(x).right=k; at(x).red=at(k).red; at(k).red=true; pull(k); pull(x); return x; }
    void flip(int k) { at(k).red=!at(k).red; at(at(k).left).red=!at(at(k).left).red; at(at(k).right).red=!at(at(k).right).red; }
    int fix(int k) {
        if(red(at(k).right)) k=left_rotate(k);
        if(red(at(k).left) && red(at(at(k).left).left)) k=right_rotate(k);
        if(red(at(k).left) && red(at(k).right)) flip(k);
        pull(k); return k;
    }
    int move_left(int k) { flip(k); if(red(at(at(k).right).left)) { at(k).right=right_rotate(at(k).right); k=left_rotate(k); flip(k); } return k; }
    int move_right(int k) { flip(k); if(red(at(at(k).left).left)) { k=right_rotate(k); flip(k); } return k; }
    int add(int k,const Key& key) {
        if(!k) { nodes_.push_back({key}); return int(nodes_.size()); }
        if(comp_(key,at(k).key)) { int x=add(at(k).left,key); at(k).left=x; }
        else if(comp_(at(k).key,key)) { int x=add(at(k).right,key); at(k).right=x; }
        else ++at(k).count;
        if(red(at(k).right) && !red(at(k).left)) k=left_rotate(k);
        if(red(at(k).left) && red(at(at(k).left).left)) k=right_rotate(k);
        if(red(at(k).left) && red(at(k).right)) flip(k);
        pull(k); return k;
    }
    int delete_min(int k) {
        if(!at(k).left) return 0;
        if(!red(at(k).left) && !red(at(at(k).left).left)) k=move_left(k);
        at(k).left=delete_min(at(k).left); return fix(k);
    }
    int remove(int k,const Key& key) {
        if(comp_(key,at(k).key)) {
            if(!red(at(k).left) && !red(at(at(k).left).left)) k=move_left(k);
            at(k).left=remove(at(k).left,key);
        } else {
            if(red(at(k).left)) k=right_rotate(k);
            bool equal=!comp_(key,at(k).key) && !comp_(at(k).key,key);
            if(equal && !at(k).right) return 0;
            if(!red(at(k).right) && !red(at(at(k).right).left)) k=move_right(k);
            equal=!comp_(key,at(k).key) && !comp_(at(k).key,key);
            if(equal) {
                int x=at(k).right; while(at(x).left) x=at(x).left;
                at(k).key=at(x).key; at(k).count=at(x).count; at(k).right=delete_min(at(k).right);
            } else at(k).right=remove(at(k).right,key);
        }
        return fix(k);
    }
    void decrement(int k,const Key& key) {
        if(comp_(key,at(k).key)) decrement(at(k).left,key);
        else if(comp_(at(k).key,key)) decrement(at(k).right,key);
        else --at(k).count;
        pull(k);
    }
public:
    explicit RedBlackTree(Compare comp=Compare{}) : comp_(comp) {}
    int size() const { return length(root_); }
    bool empty() const { return !root_; }
    void clear() { nodes_.clear(); root_=0; }
    int count(const Key& key) const { int k=root_; while(k) { if(comp_(key,at(k).key)) k=at(k).left; else if(comp_(at(k).key,key)) k=at(k).right; else return at(k).count; } return 0; }
    void insert(const Key& key) { root_=add(root_,key); at(root_).red=false; }
    bool erase(const Key& key) {
        int n=count(key); if(!n) return false;
        if(n>1) { decrement(root_,key); return true; }
        if(!red(at(root_).left) && !red(at(root_).right)) at(root_).red=true;
        root_=remove(root_,key); if(root_) at(root_).red=false; return true;
    }
    int order_of_key(const Key& key) const { int k=root_,result=0; while(k) { if(comp_(at(k).key,key)) { result+=length(at(k).left)+at(k).count; k=at(k).right; } else k=at(k).left; } return result; }
    std::optional<Key> kth(int rank) const { if(rank<0 || rank>=size()) return std::nullopt; int k=root_; while(k) { int n=length(at(k).left); if(rank<n) k=at(k).left; else if(rank<n+at(k).count) return at(k).key; else { rank-=n+at(k).count; k=at(k).right; } } return std::nullopt; }
    // 検証にも使える構造不変条件。
    bool validate() const {
        auto walk=[&](auto&& self,int k)->int {
            if(!k) return 1;
            if(red(at(k).right) || (red(k) && red(at(k).left))) return -1;
            if(at(k).size!=length(at(k).left)+at(k).count+length(at(k).right)) return -1;
            int a=self(self,at(k).left),b=self(self,at(k).right);
            return a<0 || a!=b?-1:a+(!red(k));
        };
        return !red(root_) && walk(walk,root_)>0;
    }
};
} // namespace cp
