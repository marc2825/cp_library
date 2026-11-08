#pragma once
// 説明: docs/ordered_structure/erasable_priority_queue.md
#include <cassert>
#include <cstddef>
#include <functional>
#include <map>
#include <queue>
#include <vector>

namespace cp {
// std::priority_queueと同じCompare。eraseは不在ならfalse、重複を一個削除。
template<class T,class Compare=std::less<T>> class ErasablePriorityQueue {
    std::priority_queue<T,std::vector<T>,Compare> heap_,deleted_;
    std::map<T,std::size_t,Compare> counts_; std::size_t size_=0;
    bool equal(const T& a,const T& b) const { return !counts_.key_comp()(a,b) && !counts_.key_comp()(b,a); }
    void clean() { while(!deleted_.empty() && equal(heap_.top(),deleted_.top())) { heap_.pop(); deleted_.pop(); } }
public:
    explicit ErasablePriorityQueue(Compare comp=Compare{}) : heap_(comp),deleted_(comp),counts_(comp) {}
    bool empty() const { return size_==0; }
    std::size_t size() const { return size_; }
    void push(const T& x) { heap_.push(x); ++counts_[x]; ++size_; }
    bool erase(const T& x) {
        auto it=counts_.find(x); if(it==counts_.end()) return false;
        if(--it->second==0) counts_.erase(it);
        deleted_.push(x); --size_; clean(); return true;
    }
    const T& top() { assert(!empty()); clean(); return heap_.top(); }
    void pop() { T x=top(); heap_.pop(); auto it=counts_.find(x); if(--it->second==0) counts_.erase(it); --size_; clean(); }
    void clear() { heap_=decltype(heap_)(counts_.key_comp()); deleted_=decltype(deleted_)(counts_.key_comp()); counts_.clear(); size_=0; }
};
} // namespace cp
