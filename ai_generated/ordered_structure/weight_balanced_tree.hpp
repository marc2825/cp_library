#pragma once
// 説明: docs/ordered_structure/weight_balanced_tree.md
#include <algorithm>
#include <functional>
#include <optional>
#include <vector>

namespace cp {
// 3/4-weight条件を部分木の再構築で保つ決定的BST。重複は一節点。
template<class Key,class Compare=std::less<Key>> class WeightBalancedTree {
    struct Node { Key key; int left=0,right=0,count=1,size=1,weight=1; };
    std::vector<Node> nodes_; int root_=0; Compare comp_;
    Node& at(int k) { return nodes_[k-1]; }
    const Node& at(int k) const { return nodes_[k-1]; }
    int length(int k) const { return k?at(k).size:0; }
    int weight(int k) const { return k?at(k).weight:0; }
    void pull(int k) { at(k).size=length(at(k).left)+at(k).count+length(at(k).right); at(k).weight=weight(at(k).left)+1+weight(at(k).right); }
    void flatten(int k,std::vector<int>& ids) { if(!k) return; flatten(at(k).left,ids); ids.push_back(k); flatten(at(k).right,ids); }
    int build(const std::vector<int>& ids,int l,int r) { if(l==r) return 0; int m=l+(r-l)/2,k=ids[m]; at(k).left=build(ids,l,m); at(k).right=build(ids,m+1,r); pull(k); return k; }
    int balance(int k) {
        if(!k) return 0;
        pull(k);
        if(4LL*std::max(weight(at(k).left),weight(at(k).right))>3LL*weight(k)) { std::vector<int> ids; flatten(k,ids); return build(ids,0,int(ids.size())); }
        return k;
    }
    int insert_node(int k,const Key& key) {
        if(!k) { nodes_.push_back({key}); return int(nodes_.size()); }
        if(comp_(key,at(k).key)) { int x=insert_node(at(k).left,key); at(k).left=x; }
        else if(comp_(at(k).key,key)) { int x=insert_node(at(k).right,key); at(k).right=x; }
        else ++at(k).count;
        return balance(k);
    }
    int erase_min(int k) { if(!at(k).left) return at(k).right; at(k).left=erase_min(at(k).left); return balance(k); }
    int erase_node(int k,const Key& key) {
        if(comp_(key,at(k).key)) at(k).left=erase_node(at(k).left,key);
        else if(comp_(at(k).key,key)) at(k).right=erase_node(at(k).right,key);
        else if(at(k).count>1) --at(k).count;
        else {
            if(!at(k).left) return at(k).right;
            if(!at(k).right) return at(k).left;
            int x=at(k).right; while(at(x).left) x=at(x).left;
            at(k).key=at(x).key; at(k).count=at(x).count; at(k).right=erase_min(at(k).right);
        }
        return balance(k);
    }
public:
    explicit WeightBalancedTree(Compare comp=Compare{}) : comp_(comp) {}
    int size() const { return length(root_); }
    bool empty() const { return !root_; }
    void clear() { nodes_.clear(); root_=0; }
    int count(const Key& key) const { int k=root_; while(k) { if(comp_(key,at(k).key)) k=at(k).left; else if(comp_(at(k).key,key)) k=at(k).right; else return at(k).count; } return 0; }
    void insert(const Key& key) { root_=insert_node(root_,key); }
    bool erase(const Key& key) { if(!count(key)) return false; root_=erase_node(root_,key); return true; }
    int order_of_key(const Key& key) const { int k=root_,result=0; while(k) { if(comp_(at(k).key,key)) { result+=length(at(k).left)+at(k).count; k=at(k).right; } else k=at(k).left; } return result; }
    std::optional<Key> kth(int rank) const { if(rank<0 || rank>=size()) return std::nullopt; int k=root_; while(k) { int n=length(at(k).left); if(rank<n) k=at(k).left; else if(rank<n+at(k).count) return at(k).key; else { rank-=n+at(k).count; k=at(k).right; } } return std::nullopt; }
    bool validate() const { auto walk=[&](auto&& self,int k)->bool { return !k || (at(k).size==length(at(k).left)+at(k).count+length(at(k).right) && at(k).weight==weight(at(k).left)+1+weight(at(k).right) && 4LL*std::max(weight(at(k).left),weight(at(k).right))<=3LL*weight(k) && self(self,at(k).left) && self(self,at(k).right)); }; return walk(walk,root_); }
};
} // namespace cp
