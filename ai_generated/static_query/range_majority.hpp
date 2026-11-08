#pragma once
#include <algorithm>
#include <cassert>
#include <cstddef>
#include <functional>
#include <limits>
#include <map>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

namespace cp {
// 異なる値を相殺して候補を求め、出現位置で過半数を検証する。
// 相殺結果そのものは結合的ではないため、モノイドとして公開しない。
template<class T, class Compare = std::less<T>> class RangeMajority {
    struct Vote { int index = -1, count = 0; };
    std::vector<T> values_;
    std::size_t base_ = 1;
    Compare comp_;
    std::vector<Vote> tree_;
    std::map<T,std::vector<int>,Compare> positions_;
    Vote combine(Vote a, Vote b) const {
        if (!a.count) return b;
        if (!b.count) return a;
        const T& x = values_[a.index]; const T& y = values_[b.index];
        if (!comp_(x,y) && !comp_(y,x)) return {a.index,a.count+b.count};
        if (a.count > b.count) return {a.index,a.count-b.count};
        return {b.index,b.count-a.count};
    }
public:
    explicit RangeMajority(std::vector<T> values, Compare compare = Compare{})
        : values_(std::move(values)), comp_(compare), positions_(compare) {
        if (values_.size() > std::size_t(std::numeric_limits<int>::max()))
            throw std::length_error("RangeMajority size limit");
        while (base_ < values_.size()) base_ *= 2;
        tree_.resize(2*base_);
        for (int i=0; i<size(); ++i) { tree_[base_+i] = {i,1}; positions_[values_[i]].push_back(i); }
        for (std::size_t k=base_; --k;) tree_[k] = combine(tree_[2*k],tree_[2*k+1]);
    }
    int size() const { return int(values_.size()); }
    int frequency(int l, int r, const T& value) const {
        assert(0<=l && l<=r && r<=size());
        auto it = positions_.find(value);
        if (it == positions_.end()) return 0;
        const auto& p = it->second;
        return int(std::lower_bound(p.begin(),p.end(),r)-std::lower_bound(p.begin(),p.end(),l));
    }
    std::optional<T> candidate(int l, int r) const {
        assert(0<=l && l<=r && r<=size());
        Vote a,b;
        for (std::size_t x=base_+l,y=base_+r; x<y; x/=2,y/=2) {
            if (x&1) a = combine(a,tree_[x++]);
            if (y&1) b = combine(tree_[--y],b);
        }
        Vote v = combine(a,b);
        if (!v.count) return std::nullopt;
        return values_[v.index];
    }
    std::optional<T> majority(int l, int r) const {
        auto x = candidate(l,r);
        if (x && frequency(l,r,*x) > (r-l)/2) return x;
        return std::nullopt;
    }
};
} // namespace cp
