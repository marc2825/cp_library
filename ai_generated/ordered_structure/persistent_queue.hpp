#pragma once
#include <utility>
#include <cassert>
#include <optional>
#include <vector>

namespace cp {
// 後端の永続連結列＋祖先ダブリング。分岐しても各操作O(log M)以下。
template<class T> class PersistentQueue {
    struct Node { std::optional<T> value; std::vector<int> up; };
    struct Version { int tail=0,size=0; };
    std::vector<Node> nodes_{Node{}}; std::vector<Version> versions_{Version{}};
    void check(int root) const { assert(0<=root && root<int(versions_.size())); (void)root; }
    int ancestor(int v,int count) const {
        for(int b=0;count;++b,count>>=1) if(count&1) v=nodes_[v].up[b];
        return v;
    }
public:
    int empty_root() const { return 0; }
    int size(int root) const { check(root); return versions_[root].size; }
    bool empty(int root) const { return size(root)==0; }
    T front(int root) const {
        check(root); auto v=versions_[root]; assert(v.size); return *nodes_[ancestor(v.tail,v.size-1)].value;
    }
    T back(int root) const { check(root); assert(!empty(root)); return *nodes_[versions_[root].tail].value; }
    int push(int root,const T& x) {
        check(root); auto v=versions_[root];
        // 空版からの追加では既に取り出した列を祖先に残さない。
        int tail=v.size?v.tail:0; Node node{x,{tail}};
        for(int b=1;node.up[b-1] && int(nodes_[node.up[b-1]].up.size())>=b;++b)
            node.up.push_back(nodes_[node.up[b-1]].up[b-1]);
        int k=int(nodes_.size()); nodes_.push_back(std::move(node));
        versions_.push_back({k,v.size+1}); return int(versions_.size())-1;
    }
    int pop(int root) {
        check(root); auto v=versions_[root]; assert(v.size);
        versions_.push_back({v.tail,v.size-1}); return int(versions_.size())-1;
    }
};
} // namespace cp
