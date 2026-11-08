#pragma once
#include <cassert>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <vector>
#include "../ordered_structure/dynamic_bit_vector.hpp"

namespace cp {
// 固定bit幅のuint64列。点代入・挿入・削除、頻度・順序統計。期待O(B log n)。
class DynamicWaveletMatrix {
    int bits_,size_=0; std::vector<DynamicBitVector> levels_; std::vector<int> zeros_;
    bool fits(std::uint64_t x) const { return bits_==64 || (x>>bits_)==0; }
    void bounds(int l,int r) const { assert(0<=l && l<=r && r<=size_); (void)l; (void)r; }
public:
    explicit DynamicWaveletMatrix(int bit_width=64):bits_(bit_width) {
        if(bits_<1 || bits_>64) throw std::invalid_argument("bit width must be in [1,64]");
        levels_.resize(bits_); zeros_.assign(bits_,0);
    }
    explicit DynamicWaveletMatrix(const std::vector<std::uint64_t>& values,int bit_width=64):DynamicWaveletMatrix(bit_width) {
        for(auto x:values) insert(size_,x);
    }
    int size() const { return size_; }
    bool empty() const { return size_==0; }
    int bit_width() const { return bits_; }
    void insert(int p,std::uint64_t x) {
        bounds(p,p); if(!fits(x)) throw std::out_of_range("value exceeds bit width");
        for(int d=0;d<bits_;++d) {
            bool b=(x>>(bits_-1-d))&1; int rank=levels_[d].rank(b,p);
            levels_[d].insert(p,b); p=rank+(b?zeros_[d]:0); if(!b) ++zeros_[d];
        }
        ++size_;
    }
    std::uint64_t erase(int p) {
        assert(0<=p && p<size_); std::uint64_t x=0;
        for(int d=0;d<bits_;++d) {
            bool b=levels_[d].access(p); int next=levels_[d].rank(b,p)+(b?zeros_[d]:0);
            levels_[d].erase(p); if(!b) --zeros_[d]; p=next; x=(x<<1)|b;
        }
        --size_; return x;
    }
    void set(int p,std::uint64_t x) {
        assert(0<=p && p<size_); if(!fits(x)) throw std::out_of_range("value exceeds bit width");
        erase(p); insert(p,x);
    }
    std::uint64_t access(int p) const {
        assert(0<=p && p<size_); std::uint64_t x=0;
        for(int d=0;d<bits_;++d) {
            bool b=levels_[d].access(p); x=(x<<1)|b;
            p=levels_[d].rank(b,p)+(b?zeros_[d]:0);
        }
        return x;
    }
    int count(int l,int r,std::uint64_t x) const {
        bounds(l,r); if(!fits(x)) return 0;
        for(int d=0;d<bits_;++d) {
            bool b=(x>>(bits_-1-d))&1;
            l=levels_[d].rank(b,l)+(b?zeros_[d]:0); r=levels_[d].rank(b,r)+(b?zeros_[d]:0);
        }
        return r-l;
    }
    int rank(std::uint64_t x,int end) const { return count(0,end,x); }
    int count_less(int l,int r,std::uint64_t upper) const {
        bounds(l,r); if(!fits(upper)) return r-l;
        int answer=0;
        for(int d=0;d<bits_;++d) {
            int a=levels_[d].rank(false,l),b=levels_[d].rank(false,r);
            if((upper>>(bits_-1-d))&1) { answer+=b-a; l=zeros_[d]+l-a; r=zeros_[d]+r-b; }
            else { l=a; r=b; }
        }
        return answer;
    }
    int range_freq(int l,int r,std::uint64_t lower,std::uint64_t upper) const {
        bounds(l,r); return lower<upper?count_less(l,r,upper)-count_less(l,r,lower):0;
    }
    std::optional<std::uint64_t> kth_smallest(int l,int r,int k) const {
        bounds(l,r); if(k<0 || k>=r-l) return std::nullopt;
        std::uint64_t x=0;
        for(int d=0;d<bits_;++d) {
            int a=levels_[d].rank(false,l),b=levels_[d].rank(false,r); x<<=1;
            if(k<b-a) { l=a; r=b; }
            else { k-=b-a; x|=1; l=zeros_[d]+l-a; r=zeros_[d]+r-b; }
        }
        return x;
    }
    int select(std::uint64_t x,int occurrence) const {
        if(!fits(x) || occurrence<0) return -1;
        int l=0,r=size_;
        for(int d=0;d<bits_;++d) {
            bool b=(x>>(bits_-1-d))&1;
            l=levels_[d].rank(b,l)+(b?zeros_[d]:0); r=levels_[d].rank(b,r)+(b?zeros_[d]:0);
        }
        if(occurrence>=r-l) return -1;
        int p=l+occurrence;
        for(int d=bits_-1;d>=0;--d) {
            bool b=(x>>(bits_-1-d))&1; p=levels_[d].select(b,p-(b?zeros_[d]:0));
        }
        return p;
    }
    std::optional<std::uint64_t> predecessor(int l,int r,std::uint64_t upper) const {
        return kth_smallest(l,r,count_less(l,r,upper)-1);
    }
    std::optional<std::uint64_t> successor(int l,int r,std::uint64_t lower) const {
        return kth_smallest(l,r,count_less(l,r,lower));
    }
};
} // namespace cp
