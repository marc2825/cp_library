#pragma once
#include <algorithm>
#include <array>
#include <cassert>
#include <cstddef>
#include <functional>
#include <limits>
#include <utility>
#include <vector>

namespace cp {
// 根=1、葉=base()+p。標準ノードを左から右へ列挙する。実体の配列は持たない。
class SegmentTreeCover {
    int n_;
    std::size_t base_=1;
public:
    explicit SegmentTreeCover(int n) : n_(n) {
        assert(n>=0);
        while (base_<std::size_t(n)) base_*=2;
    }
    int size() const { return n_; }
    std::size_t base() const { return base_; }
    std::size_t node_capacity() const { return 2*base_; }
    std::size_t leaf(int p) const { assert(0<=p && p<n_); return base_+p; }
    template<class F> void for_each(int l, int r, F&& visit) const {
        assert(0<=l && l<=r && r<=n_);
        std::array<std::size_t,std::numeric_limits<std::size_t>::digits> right{};
        std::size_t count=0;
        for (std::size_t a=base_+l,b=base_+r; a<b; a/=2,b/=2) {
            if (a&1) std::invoke(visit,a++);
            if (b&1) right[count++]=--b;
        }
        while (count) std::invoke(visit,right[--count]);
    }
    std::vector<std::size_t> cover(int l, int r) const {
        std::vector<std::size_t> nodes;
        for_each(l,r,[&](std::size_t k){ nodes.push_back(k); });
        return nodes;
    }
    // padding を [0,n) へ切り詰める。padding だけのノードは [n,n)。
    std::pair<int,int> interval(std::size_t k) const {
        assert(0<k && k<node_capacity());
        std::size_t width=base_,level=1;
        while (2*level<=k) { level*=2; width/=2; }
        std::size_t l=(k-level)*width;
        return {int(std::min(l,std::size_t(n_))),int(std::min(l+width,std::size_t(n_)))};
    }
};
} // namespace cp
