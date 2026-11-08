#pragma once
#include <algorithm>
#include <cassert>
#include <cstddef>
#include <functional>
#include <list>
#include <utility>
#include <vector>

namespace cp { namespace detail {
// listのspliceで所有権をO(1)移譲。節点破棄は再帰しない。
template<class T,class Compare,bool Leftist> class MeldableHeap {
    struct Node { T value; Node* left=nullptr; Node* right=nullptr; int rank=1; };
    std::list<Node> nodes_; Node* root_=nullptr; Compare comp_; std::size_t size_=0;
    static int rank(Node* p) { return p?p->rank:0; }
    Node* merge(Node* a,Node* b) {
        std::vector<Node*> path;
        while(a && b) {
            if(comp_(a->value,b->value)) std::swap(a,b);
            path.push_back(a); a=a->right;
        }
        Node* result=a?a:b;
        while(!path.empty()) {
            Node* p=path.back(); path.pop_back(); p->right=result;
            if(!Leftist || rank(p->left)<rank(p->right)) std::swap(p->left,p->right);
            p->rank=1+rank(p->right); result=p;
        }
        return result;
    }
public:
    explicit MeldableHeap(Compare comp=Compare{}) : comp_(comp) {}
    MeldableHeap(const MeldableHeap&)=delete;
    MeldableHeap& operator=(const MeldableHeap&)=delete;
    MeldableHeap(MeldableHeap&& other) : comp_(std::move(other.comp_)) { nodes_.splice(nodes_.end(),other.nodes_); root_=other.root_; size_=other.size_; other.root_=nullptr; other.size_=0; }
    MeldableHeap& operator=(MeldableHeap&& other) {
        if(this!=&other) { clear(); comp_=std::move(other.comp_); nodes_.splice(nodes_.end(),other.nodes_); root_=other.root_; size_=other.size_; other.root_=nullptr; other.size_=0; } return *this;
    }
    bool empty() const { return !size_; }
    std::size_t size() const { return size_; }
    const T& top() const { assert(root_); return root_->value; }
    void push(const T& x) { nodes_.push_back({x}); root_=merge(root_,&nodes_.back()); ++size_; }
    void pop() { assert(root_); root_=merge(root_->left,root_->right); --size_; }
    // 同じ順序を定める比較器同士。otherは空になる。
    void meld(MeldableHeap& other) { if(this==&other) return; root_=merge(root_,other.root_); nodes_.splice(nodes_.end(),other.nodes_); size_+=other.size_; other.root_=nullptr; other.size_=0; }
    void clear() { root_=nullptr; size_=0; nodes_.clear(); }
};
} } // namespace cp::detail
