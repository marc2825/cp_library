#pragma once
#include <algorithm>
#include <cassert>
#include <cstddef>
#include <functional>
#include <limits>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

namespace cp {
// pool 所有の永続 heap。root=0 は空。Compare は既存 LeftistHeap と同じ向き。
template<class T,class Compare=std::less<T>> class PersistentLeftistHeap {
    struct Node {
        std::optional<T> value;
        int left=0,right=0,rank=0;
        std::size_t size=0;
    };
    std::vector<Node> nodes_{Node{}};
    Compare comp_;
    void check(int root) const { assert(0<=root && std::size_t(root)<nodes_.size()); (void)root; }
    int make(Node node) {
        if (nodes_.size()>=std::size_t(std::numeric_limits<int>::max()))
            throw std::length_error("PersistentLeftistHeap node limit");
        nodes_.push_back(std::move(node)); return int(nodes_.size())-1;
    }
    int join(int a,int b,bool copy) {
        if (!a || !b) return a?a:b;
        if (comp_(*nodes_[a].value,*nodes_[b].value)) std::swap(a,b);
        if (copy) a=make(nodes_[a]);
        int child=join(nodes_[a].right,b,copy);
        nodes_[a].right=child;
        auto& n=nodes_[a];
        if (nodes_[n.left].rank<nodes_[n.right].rank) std::swap(n.left,n.right);
        n.rank=nodes_[n.right].rank+1;
        n.size=1+nodes_[n.left].size+nodes_[n.right].size;
        return a;
    }
public:
    explicit PersistentLeftistHeap(Compare compare=Compare{}) : comp_(compare) {}
    int empty_root() const { return 0; }
    bool empty(int root) const { check(root); return root==0; }
    std::size_t size(int root) const { check(root); return nodes_[root].size; }
    std::size_t node_count() const { return nodes_.size()-1; }
    const T& top(int root) const { check(root); assert(root); return *nodes_[root].value; }
    std::pair<int,int> children(int root) const { check(root); return {nodes_[root].left,nodes_[root].right}; }
    int meld(int a,int b) {
        check(a); check(b);
        if (size(a)>std::numeric_limits<std::size_t>::max()-size(b))
            throw std::length_error("PersistentLeftistHeap size limit");
        return join(a,b,true);
    }
    int push(int root,T value) {
        check(root);
        if (size(root)==std::numeric_limits<std::size_t>::max())
            throw std::length_error("PersistentLeftistHeap size limit");
        return join(root,make({std::move(value),0,0,1,1}),true);
    }
    int pop(int root) { check(root); assert(root); return meld(nodes_[root].left,nodes_[root].right); }
    // 新規ノードだけを破壊的に pairwise meld するため O(n) 構築。
    template<class It> int build(It first,It last) {
        std::vector<int> roots;
        for (;first!=last;++first) roots.push_back(make({*first,0,0,1,1}));
        while (roots.size()>1) {
            std::size_t next=0;
            for (std::size_t i=0;i<roots.size();i+=2)
                roots[next++]=i+1<roots.size()?join(roots[i],roots[i+1],false):roots[i];
            roots.resize(next);
        }
        return roots.empty()?0:roots[0];
    }
};
} // namespace cp
