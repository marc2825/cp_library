#pragma once
// 説明: docs/static_query/linear_rmq.md
#include <algorithm>
#include <cassert>
#include <cstdint>
#include <functional>
#include <vector>

namespace cp {
// 小ブロックの単調stackをbitmask化。O(n)構築/領域、O(1) argmin。
template<class T,class Compare=std::less<T>> class LinearRMQ {
    std::vector<T> values_; Compare comp_; int width_=1;
    std::vector<std::uint64_t> masks_; std::vector<std::vector<int>> sparse_;
    int better(int a,int b) const { if(a<0) return b; if(b<0) return a; return comp_(values_[b],values_[a])?b:comp_(values_[a],values_[b])?a:std::min(a,b); }
    int inside(int l,int r) const { auto bits=masks_[r-1]&(~std::uint64_t(0)<<(l%width_)); return (l/width_)*width_+__builtin_ctzll(bits); }
public:
    explicit LinearRMQ(const std::vector<T>& a={},Compare comp=Compare{}) : values_(a),comp_(comp),masks_(a.size()) {
        int log=0; for(std::size_t n=a.size();n>1;n/=2) ++log;
        width_=std::max(1,log/2); int blocks=(size()+width_-1)/width_;
        std::vector<int> minima(blocks,-1);
        for(int i=0;i<size();++i) {
            std::uint64_t mask=i%width_?masks_[i-1]:0;
            while(mask) { int p=63-__builtin_clzll(mask),j=i/width_*width_+p; if(!comp_(a[i],a[j])) break; mask^=std::uint64_t(1)<<p; }
            masks_[i]=mask|(std::uint64_t(1)<<(i%width_)); minima[i/width_]=better(minima[i/width_],i);
        }
        sparse_.push_back(minima);
        for(int k=1;(1LL<<k)<=blocks;++k) {
            sparse_.push_back(std::vector<int>(blocks-(1<<k)+1));
            for(int i=0;i<int(sparse_.back().size());++i) sparse_[k][i]=better(sparse_[k-1][i],sparse_[k-1][i+(1<<(k-1))]);
        }
    }
    int size() const { return int(values_.size()); }
    int argmin(int l,int r) const {
        assert(0<=l && l<=r && r<=size()); if(l==r) return -1;
        if(l/width_==(r-1)/width_) return inside(l,r);
        int a=l/width_+1,b=(r-1)/width_;
        int best=better(inside(l,a*width_),inside(b*width_,r));
        if(a<b) { int k=31-__builtin_clz(unsigned(b-a)); best=better(best,better(sparse_[k][a],sparse_[k][b-(1<<k)])); }
        return best;
    }
};
} // namespace cp
