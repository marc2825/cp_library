#pragma once
// 説明: docs/static_query/weighted_wavelet_matrix.md
#include <algorithm>
#include <cassert>
#include <cstdint>
#include <utility>
#include <vector>
#include "succinct_bit_vector.hpp"

namespace cp {
// key順に重みを保持。値域和と小さいk個の重み和。重みは負でも可。
template<class Key,class Weight=long long> class WeightedWaveletMatrix {
    int n_,levels_=0; std::vector<Key> keys_; std::vector<SuccinctBitVector> bits_;
    std::vector<int> zeros_; std::vector<std::vector<Weight>> prefix_; std::vector<Weight> original_;
    void bounds(int l,int r) const { assert(0<=l && l<=r && r<=n_); (void)l; (void)r; }
    Weight less(int l,int r,int x) const {
        if(x>=int(keys_.size())) return original_[r]-original_[l];
        Weight result=Weight(0);
        for(int d=0;d<levels_;++d) {
            int a=bits_[d].rank(l),b=bits_[d].rank(r),zl=l-a,zr=r-b;
            if((x>>(levels_-1-d))&1) { result=result+prefix_[d][zr]-prefix_[d][zl]; l=zeros_[d]+a; r=zeros_[d]+b; }
            else { l=zl; r=zr; }
        }
        return result;
    }
public:
    WeightedWaveletMatrix(const std::vector<Key>& keys,const std::vector<Weight>& weights) : n_(int(keys.size())),keys_(keys),original_(n_+1,Weight(0)) {
        assert(keys.size()==weights.size()); std::sort(keys_.begin(),keys_.end());
        keys_.erase(std::unique(keys_.begin(),keys_.end(),[](const Key& a,const Key& b){return !(a<b)&&!(b<a);}),keys_.end());
        while((std::uint64_t(1)<<levels_)<keys_.size()) ++levels_;
        std::vector<int> cur(n_); std::vector<Weight> w=weights;
        for(int i=0;i<n_;++i) { cur[i]=int(std::lower_bound(keys_.begin(),keys_.end(),keys[i])-keys_.begin()); original_[i+1]=original_[i]+weights[i]; }
        for(int b=levels_-1;b>=0;--b) {
            SuccinctBitVector bits(n_); std::vector<int> next; std::vector<Weight> nw;
            for(int i=0;i<n_;++i) if((cur[i]>>b)&1) bits.set(i);
            bits.build(); bits_.push_back(bits); zeros_.push_back(n_-bits.rank(n_));
            for(int side=0;side<2;++side) for(int i=0;i<n_;++i) if(((cur[i]>>b)&1)==side) { next.push_back(cur[i]); nw.push_back(w[i]); }
            std::vector<Weight> prefix(n_+1,Weight(0));
            for(int i=0;i<n_;++i) prefix[i+1]=prefix[i]+nw[i];
            prefix_.push_back(std::move(prefix)); cur.swap(next); w.swap(nw);
        }
        // 全key同値の場合にもstableな順を残す。
        if(!levels_) prefix_.push_back(original_);
    }
    int size() const { return n_; }
    Weight sum_less(int l,int r,const Key& upper) const { bounds(l,r); return less(l,r,int(std::lower_bound(keys_.begin(),keys_.end(),upper)-keys_.begin())); }
    Weight range_sum(int l,int r,const Key& lower,const Key& upper) const { bounds(l,r); if(!(lower<upper)) return Weight(0); return sum_less(l,r,upper)-sum_less(l,r,lower); }
    Weight sum_smallest(int l,int r,int k) const {
        bounds(l,r); assert(0<=k && k<=r-l); Weight result=Weight(0);
        for(int d=0;d<levels_;++d) {
            int a=bits_[d].rank(l),b=bits_[d].rank(r),zl=l-a,zr=r-b,count=zr-zl;
            if(k<count) { l=zl; r=zr; }
            else { result=result+prefix_[d][zr]-prefix_[d][zl]; k-=count; l=zeros_[d]+a; r=zeros_[d]+b; }
        }
        const auto& prefix=prefix_.back(); return result+prefix[l+k]-prefix[l];
    }
};
} // namespace cp
