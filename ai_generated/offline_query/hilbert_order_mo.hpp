#pragma once
// 説明: docs/offline_query/hilbert_order_mo.md
#include <algorithm>
#include <cassert>
#include <cstdint>
#include <numeric>
#include <utility>
#include <vector>

namespace cp {
inline std::uint64_t hilbert_order(std::uint32_t x,std::uint32_t y,unsigned bits=31) {
    assert(bits<=31 && (std::uint64_t(x)>>bits)==0 && (std::uint64_t(y)>>bits)==0);
    if(!bits) return 0;
    std::uint32_t half=std::uint32_t(1)<<(bits-1);
    unsigned quadrant=x<half?(y<half?0:3):(y<half?1:2);
    x&=half-1; y&=half-1;
    if(quadrant==0) std::swap(x,y);
    if(quadrant==3) { auto a=x; x=half-1-y; y=half-1-a; }
    return std::uint64_t(quadrant)*(std::uint64_t(1)<<(2*bits-2))+hilbert_order(x,y,bits-1);
}
class HilbertOrderMo {
    int n_; std::vector<std::pair<int,int>> queries_;
public:
    explicit HilbertOrderMo(int n) : n_(n) { assert(n>=0); }
    int add_query(int l,int r) { assert(0<=l && l<=r && r<=n_); queries_.emplace_back(l,r); return query_count()-1; }
    int query_count() const { return int(queries_.size()); }
    std::vector<int> query_order() const {
        std::vector<int> order(query_count()); std::iota(order.begin(),order.end(),0);
        std::vector<std::uint64_t> keys; for(auto [l,r]:queries_) keys.push_back(hilbert_order(unsigned(l),unsigned(r)));
        std::stable_sort(order.begin(),order.end(),[&](int a,int b){return keys[a]<keys[b];}); return order;
    }
    template<class AL,class AR,class RL,class RR,class Answer>
    void run(AL al,AR ar,RL rl,RR rr,Answer answer) const {
        int l=0,r=0;
        for(int id:query_order()) {
            auto [a,b]=queries_[id];
            while(l>a) al(--l);
            while(r<b) ar(r++);
            while(l<a) rl(l++);
            while(r>b) rr(--r);
            answer(id);
        }
    }
    template<class Add,class Remove,class Answer> void run(Add add,Remove remove,Answer answer) const { run(add,add,remove,remove,answer); }
};
} // namespace cp
