#pragma once
// 説明: docs/basic_utility/sqrt_decomposition.md
#include <algorithm>
#include <cassert>
#include <cmath>
#include <vector>

namespace cp {
// モノイドの平方分割。set/prod O(√n)、構築O(n)。非可換も可。
template<class S, auto op, auto e> class SqrtDecomposition {
    std::vector<S> values_, blocks_; int width_;
    void rebuild(int b) {
        S x=e();
        for (int i=b*width_; i<std::min(size(),(b+1)*width_); ++i) x=op(x,values_[i]);
        blocks_[b]=x;
    }
public:
    explicit SqrtDecomposition(const std::vector<S>& a={}) : values_(a), width_(std::max(1,int(std::sqrt(a.size())))) {
        blocks_.assign((size()+width_-1)/width_,e());
        for (int b=0;b<int(blocks_.size());++b) rebuild(b);
    }
    int size() const { return int(values_.size()); }
    S get(int p) const { assert(0<=p && p<size()); return values_[p]; }
    void set(int p,const S& x) { assert(0<=p && p<size()); values_[p]=x; rebuild(p/width_); }
    S prod(int l,int r) const {
        assert(0<=l && l<=r && r<=size()); S result=e();
        while (l<r && l%width_) result=op(result,values_[l++]);
        while (r-l>=width_) { result=op(result,blocks_[l/width_]); l+=width_; }
        while (l<r) result=op(result,values_[l++]);
        return result;
    }
};
} // namespace cp
