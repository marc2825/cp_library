#pragma once
// 説明: docs/static_query/dynamic_bitset.md
#include <algorithm>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace cp {
class DynamicBitset {
    std::size_t n_; std::vector<std::uint64_t> bits_;
    void trim() { if(n_%64) bits_.back()&=(std::uint64_t(1)<<(n_%64))-1; }
public:
    explicit DynamicBitset(std::size_t n=0,bool value=false) : n_(n),bits_(n/64+(n%64!=0),value?~std::uint64_t(0):0) { trim(); }
    std::size_t size() const { return n_; }
    bool test(std::size_t p) const { assert(p<n_); return (bits_[p/64]>>(p%64))&1; }
    void set(std::size_t p,bool value=true) { assert(p<n_); auto mask=std::uint64_t(1)<<(p%64); if(value) bits_[p/64]|=mask; else bits_[p/64]&=~mask; }
    void flip(std::size_t p) { assert(p<n_); bits_[p/64]^=std::uint64_t(1)<<(p%64); }
    void reset() { std::fill(bits_.begin(),bits_.end(),0); }
    std::size_t count() const { std::size_t n=0; for(auto x:bits_) n+=__builtin_popcountll(x); return n; }
    bool any() const { return std::any_of(bits_.begin(),bits_.end(),[](auto x){return x!=0;}); }
    DynamicBitset& operator&=(const DynamicBitset& b) { assert(n_==b.n_); for(std::size_t i=0;i<bits_.size();++i) bits_[i]&=b.bits_[i]; return *this; }
    DynamicBitset& operator|=(const DynamicBitset& b) { assert(n_==b.n_); for(std::size_t i=0;i<bits_.size();++i) bits_[i]|=b.bits_[i]; return *this; }
    DynamicBitset& operator^=(const DynamicBitset& b) { assert(n_==b.n_); for(std::size_t i=0;i<bits_.size();++i) bits_[i]^=b.bits_[i]; return *this; }
    friend DynamicBitset operator&(DynamicBitset a,const DynamicBitset& b) { return a&=b; }
    friend DynamicBitset operator|(DynamicBitset a,const DynamicBitset& b) { return a|=b; }
    friend DynamicBitset operator^(DynamicBitset a,const DynamicBitset& b) { return a^=b; }
    DynamicBitset operator~() const { DynamicBitset r=*this; for(auto& x:r.bits_) x=~x; r.trim(); return r; }
    DynamicBitset operator<<(std::size_t shift) const {
        DynamicBitset r(n_); if(shift>=n_) return r;
        std::size_t q=shift/64; unsigned s=unsigned(shift%64);
        for(std::size_t i=q;i<bits_.size();++i) { r.bits_[i]=bits_[i-q]<<s; if(s && i>q) r.bits_[i]|=bits_[i-q-1]>>(64-s); }
        r.trim(); return r;
    }
    DynamicBitset operator>>(std::size_t shift) const {
        DynamicBitset r(n_); if(shift>=n_) return r;
        std::size_t q=shift/64; unsigned s=unsigned(shift%64);
        for(std::size_t i=0;i+q<bits_.size();++i) { r.bits_[i]=bits_[i+q]>>s; if(s && i+q+1<bits_.size()) r.bits_[i]|=bits_[i+q+1]<<(64-s); }
        return r;
    }
    // p以上の最初の1。未発見はsize()。
    std::size_t find_next(std::size_t p=0) const {
        if(p>=n_) return n_;
        std::size_t i=p/64; auto x=bits_[i]&(~std::uint64_t(0)<<(p%64));
        while(!x) { if(++i==bits_.size()) return n_; x=bits_[i]; }
        return i*64+__builtin_ctzll(x);
    }
};
} // namespace cp
