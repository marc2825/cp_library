#pragma once
#include <cassert>
#include <cstddef>
#include <functional>
#include <limits>
#include <map>
#include <stdexcept>
#include <utility>
#include <vector>

namespace cp {
// 各標準区間に値別の個数を保持。未知の値への set もオンラインで扱う。
template<class T, class Compare = std::less<T>> class PointSetRangeFrequency {
    std::vector<T> values_;
    std::size_t base_ = 1;
    Compare comp_;
    std::vector<std::map<T,int,Compare>> tree_;
    static int count(const std::map<T,int,Compare>& m, const T& x) {
        auto it=m.find(x); return it==m.end()?0:it->second;
    }
public:
    explicit PointSetRangeFrequency(std::vector<T> values, Compare compare = Compare{})
        : values_(std::move(values)), comp_(compare) {
        if (values_.size() > std::size_t(std::numeric_limits<int>::max()))
            throw std::length_error("PointSetRangeFrequency size limit");
        while (base_ < values_.size()) base_ *= 2;
        tree_.assign(2*base_,std::map<T,int,Compare>(compare));
        for (int i=0; i<size(); ++i)
            for (std::size_t k=base_+i; k; k/=2) ++tree_[k][values_[i]];
    }
    int size() const { return int(values_.size()); }
    typename std::vector<T>::const_reference get(int p) const { assert(0<=p && p<size()); return values_[p]; }
    void set(int p, T value) {
        assert(0<=p && p<size());
        if (!comp_(values_[p],value) && !comp_(value,values_[p])) { values_[p]=std::move(value); return; }
        for (std::size_t k=base_+p; k; k/=2) {
            auto it=tree_[k].find(values_[p]);
            if (--it->second == 0) tree_[k].erase(it);
            ++tree_[k][value];
        }
        values_[p]=std::move(value);
    }
    int frequency(int l, int r, const T& value) const {
        assert(0<=l && l<=r && r<=size());
        int answer=0;
        for (std::size_t a=base_+l,b=base_+r; a<b; a/=2,b/=2) {
            if (a&1) answer+=count(tree_[a++],value);
            if (b&1) answer+=count(tree_[--b],value);
        }
        return answer;
    }
};
} // namespace cp
