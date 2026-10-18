#pragma once
// 説明: docs/static_query/priority_sum.md
#include <algorithm>
#include <cstddef>
#include <functional>
#include <iterator>
#include <set>

namespace cp {
// 比較器順の先頭k個（不足時は全体）の和。既定は小さいk個。
template<class T,class Compare=std::less<T>> class PrioritySum {
    std::multiset<T,Compare> selected_,rest_; std::size_t k_; T sum_=T(0);
    void balance() {
        std::size_t target=std::min(k_,size());
        while(selected_.size()>target) { auto it=std::prev(selected_.end()); sum_=sum_-*it; rest_.insert(*it); selected_.erase(it); }
        while(selected_.size()<target) { auto it=rest_.begin(); sum_=sum_+*it; selected_.insert(*it); rest_.erase(it); }
        while(!selected_.empty() && !rest_.empty() && selected_.key_comp()(*rest_.begin(),*std::prev(selected_.end()))) {
            auto a=std::prev(selected_.end()),b=rest_.begin(); T x=*a,y=*b;
            selected_.erase(a); rest_.erase(b); selected_.insert(y); rest_.insert(x); sum_=sum_-x+y;
        }
    }
public:
    explicit PrioritySum(std::size_t k=0,Compare comp=Compare{}) : selected_(comp),rest_(comp),k_(k) {}
    std::size_t size() const { return selected_.size()+rest_.size(); }
    std::size_t get_k() const { return k_; }
    void set_k(std::size_t k) { k_=k; balance(); }
    T sum() const { return sum_; }
    void insert(const T& x) { rest_.insert(x); balance(); }
    bool erase(const T& x) {
        auto it=selected_.find(x);
        if(it!=selected_.end()) { sum_=sum_-*it; selected_.erase(it); }
        else { auto jt=rest_.find(x); if(jt==rest_.end()) return false; rest_.erase(jt); }
        balance(); return true;
    }
};
} // namespace cp
