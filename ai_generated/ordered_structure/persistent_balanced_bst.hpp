#pragma once
// 説明: docs/ordered_structure/persistent_balanced_bst.md
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <utility>

namespace cp {
// 不変節点を共有する永続Treap多重集合。オブジェクトのコピーはO(1)。
template<class Key,class Compare=std::less<Key>> class PersistentBalancedBST {
    struct Node;
    using Pointer=std::shared_ptr<const Node>;
    struct Node { Key key; std::uint64_t priority; int count,size; Pointer left,right; };
    Pointer root_; Compare comp_; std::uint64_t state_;
    static int length(Pointer p) { return p?p->size:0; }
    Pointer make(const Key& key,std::uint64_t priority,int count,Pointer left,Pointer right) const { return std::make_shared<Node>(Node{key,priority,count,count+length(left)+length(right),left,right}); }
    std::uint64_t random() { state_+=0x9e3779b97f4a7c15ULL; auto x=state_; x=(x^(x>>30))*0xbf58476d1ce4e5b9ULL; x=(x^(x>>27))*0x94d049bb133111ebULL; return x^(x>>31); }
    Pointer merge(Pointer a,Pointer b) const {
        if(!a || !b) return a?a:b;
        if(a->priority>b->priority) return make(a->key,a->priority,a->count,a->left,merge(a->right,b));
        return make(b->key,b->priority,b->count,merge(a,b->left),b->right);
    }
    Pointer add(Pointer p,const Key& key,std::uint64_t priority) const {
        if(!p) return make(key,priority,1,nullptr,nullptr);
        if(!comp_(key,p->key) && !comp_(p->key,key)) return make(p->key,p->priority,p->count+1,p->left,p->right);
        if(comp_(key,p->key)) {
            auto x=add(p->left,key,priority);
            if(x->priority>p->priority) return make(x->key,x->priority,x->count,x->left,make(p->key,p->priority,p->count,x->right,p->right));
            return make(p->key,p->priority,p->count,x,p->right);
        }
        auto x=add(p->right,key,priority);
        if(x->priority>p->priority) return make(x->key,x->priority,x->count,make(p->key,p->priority,p->count,p->left,x->left),x->right);
        return make(p->key,p->priority,p->count,p->left,x);
    }
    Pointer remove(Pointer p,const Key& key) const {
        if(!p) return p;
        if(comp_(key,p->key)) return make(p->key,p->priority,p->count,remove(p->left,key),p->right);
        if(comp_(p->key,key)) return make(p->key,p->priority,p->count,p->left,remove(p->right,key));
        return p->count>1?make(p->key,p->priority,p->count-1,p->left,p->right):merge(p->left,p->right);
    }
public:
    explicit PersistentBalancedBST(Compare comp=Compare{},std::uint64_t seed=712367821) : comp_(comp),state_(seed) {}
    int size() const { return length(root_); }
    int count(const Key& key) const { auto p=root_; while(p) { if(comp_(key,p->key)) p=p->left; else if(comp_(p->key,key)) p=p->right; else return p->count; } return 0; }
    PersistentBalancedBST insert(const Key& key) const { auto next=*this; auto priority=next.random(); next.root_=next.add(root_,key,priority); return next; }
    PersistentBalancedBST erase(const Key& key) const { auto next=*this; if(count(key)) next.root_=remove(root_,key); return next; }
    int order_of_key(const Key& key) const { auto p=root_; int result=0; while(p) { if(comp_(p->key,key)) { result+=length(p->left)+p->count; p=p->right; } else p=p->left; } return result; }
    std::optional<Key> kth(int rank) const { if(rank<0 || rank>=size()) return std::nullopt; auto p=root_; while(p) { int n=length(p->left); if(rank<n) p=p->left; else if(rank<n+p->count) return p->key; else { rank-=n+p->count; p=p->right; } } return std::nullopt; }
};
} // namespace cp
