#pragma once
// 説明: docs/static_query/succinct_bit_vector.md
#include <cassert>
#include <cstdint>
#include <vector>

namespace cp {
// packed bits + block prefix。rank O(1)、select O(log n)。build後のみ検索可。
class SuccinctBitVector {
    int n_; std::vector<std::uint64_t> bits_; std::vector<int> prefix_; bool built_=false;
public:
    explicit SuccinctBitVector(int n=0) : n_(n),bits_((n+63LL)/64),prefix_(bits_.size()+1) { assert(n>=0); }
    int size() const { return n_; }
    void set(int p,bool value=true) { assert(0<=p && p<n_); auto mask=std::uint64_t(1)<<(p%64); if(value) bits_[p/64]|=mask; else bits_[p/64]&=~mask; built_=false; }
    bool get(int p) const { assert(0<=p && p<n_); return (bits_[p/64]>>(p%64))&1; }
    void build() { prefix_[0]=0; for(std::size_t i=0;i<bits_.size();++i) prefix_[i+1]=prefix_[i]+__builtin_popcountll(bits_[i]); built_=true; }
    int rank(int r,bool value=true) const {
        assert(built_ && 0<=r && r<=n_);
        int ones=prefix_[r/64]+(r%64?__builtin_popcountll(bits_[r/64]&((std::uint64_t(1)<<(r%64))-1)):0);
        return value?ones:r-ones;
    }
    int select(int k,bool value=true) const {
        assert(built_); if(k<0 || k>=rank(n_,value)) return -1;
        int l=0,r=n_;
        while(l<r) { int m=l+(r-l)/2; if(rank(m+1,value)>k) r=m; else l=m+1; }
        return l;
    }
};
} // namespace cp
