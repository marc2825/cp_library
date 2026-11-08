#pragma once
#include <algorithm>
#include <cassert>
#include <cstdint>
#include <optional>
#include <vector>

namespace cp {
// 座標圧縮した静的 Wavelet Matrix。構築 O(n log n)、検索 O(log n)。
// 説明: docs/static_query/wavelet_matrix.md
template<class T> class WaveletMatrix {
    struct Bits {
        std::vector<std::uint64_t> data; std::vector<int> prefix;
        explicit Bits(int n=0):data((n+63LL)/64),prefix(data.size()+1){}
        void set(int i) { data[i/64]|=std::uint64_t(1)<<(i%64); }
        static int popcount(std::uint64_t x) {
            x -= (x>>1)&0x5555555555555555ULL;
            x = (x&0x3333333333333333ULL)+((x>>2)&0x3333333333333333ULL);
            x = (x+(x>>4))&0x0f0f0f0f0f0f0f0fULL;
            return int((x*0x0101010101010101ULL)>>56);
        }
        void build() { for(std::size_t i=0;i<data.size();++i) prefix[i+1]=prefix[i]+popcount(data[i]); }
        int rank(int r) const { return prefix[r/64]+(r%64?popcount(data[r/64]&((std::uint64_t(1)<<(r%64))-1)):0); }
    };
    int n_=0, levels_=0; std::vector<T> values_; std::vector<Bits> bits_; std::vector<int> zeros_;
    void bounds(int l,int r) const { assert(0<=l && l<=r && r<=n_); (void)l; (void)r; }
    int less_rank(int l,int r,int x) const {
        if(x>=int(values_.size())) return r-l;
        int result=0;
        for(int d=0;d<levels_;++d) {
            int a=bits_[d].rank(l), b=bits_[d].rank(r);
            if((x>>(levels_-1-d))&1) { result+=(r-l)-(b-a); l=zeros_[d]+a; r=zeros_[d]+b; }
            else { l-=a; r-=b; }
        }
        return result;
    }
public:
    explicit WaveletMatrix(const std::vector<T>& a={}) : n_(int(a.size())),values_(a) {
        std::sort(values_.begin(),values_.end());
        values_.erase(std::unique(values_.begin(),values_.end(),[](const T& x,const T& y){return !(x<y)&&!(y<x);}),values_.end());
        while((std::uint64_t(1)<<levels_)<values_.size()) ++levels_;
        std::vector<int> cur(n_),next(n_);
        for(int i=0;i<n_;++i) cur[i]=int(std::lower_bound(values_.begin(),values_.end(),a[i])-values_.begin());
        for(int b=levels_-1;b>=0;--b) {
            bits_.emplace_back(n_); int z=0;
            for(int i=0;i<n_;++i) { if((cur[i]>>b)&1) bits_.back().set(i); else ++z; }
            bits_.back().build(); zeros_.push_back(z); int p=0,q=z;
            for(int v:cur) next[(v>>b)&1?q++:p++]=v;
            cur.swap(next);
        }
    }
    int size() const { return n_; }
    std::optional<T> kth_smallest(int l,int r,int k) const {
        bounds(l,r); if(k<0 || k>=r-l) return std::nullopt; int x=0;
        for(int d=0;d<levels_;++d) {
            int a=bits_[d].rank(l), b=bits_[d].rank(r), z=(r-l)-(b-a);
            x*=2;
            if(k<z) { l-=a; r-=b; } else { k-=z; ++x; l=zeros_[d]+a; r=zeros_[d]+b; }
        }
        return values_[x];
    }
    T access(int p) const { assert(0<=p && p<n_); return *kth_smallest(p,p+1,0); }
    int count_less(int l,int r,const T& x) const { bounds(l,r); return less_rank(l,r,int(std::lower_bound(values_.begin(),values_.end(),x)-values_.begin())); }
    int count(int l,int r,const T& x) const {
        bounds(l,r); int a=int(std::lower_bound(values_.begin(),values_.end(),x)-values_.begin());
        int b=int(std::upper_bound(values_.begin(),values_.end(),x)-values_.begin()); return less_rank(l,r,b)-less_rank(l,r,a);
    }
    int range_freq(int l,int r,const T& lower,const T& upper) const { return upper<lower?0:count_less(l,r,upper)-count_less(l,r,lower); }
    std::optional<T> predecessor(int l,int r,const T& x) const { return kth_smallest(l,r,count_less(l,r,x)-1); }
    std::optional<T> successor(int l,int r,const T& x) const { return kth_smallest(l,r,count_less(l,r,x)); }
};
} // namespace cp
