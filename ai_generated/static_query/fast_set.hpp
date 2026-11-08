#pragma once
#include <algorithm>
#include <cassert>
#include <cstdint>
#include <vector>

namespace cp {
// 固定宇宙[0,n)。階層64bit集合。next/prevは境界を含み、なしはn/-1。
class FastSet {
    int n_,size_=0; std::vector<std::vector<std::uint64_t>> levels_;
    static int low(std::uint64_t x) { assert(x); int b=0; while(!(x&1)) { x>>=1; ++b; } return b; }
    static int high(std::uint64_t x) { assert(x); int b=0; while(x>>=1) ++b; return b; }
public:
    explicit FastSet(int n):n_(n) {
        assert(0<=n && n<=100000000);
        int words=std::max(1,(n+63)/64);
        for (;;) { levels_.emplace_back(words,0); if(words==1) break; words=(words+63)/64; }
    }
    int size() const { return size_; }
    bool contains(int x) const { return 0<=x && x<n_ && ((levels_[0][x/64]>>(x%64))&1); }
    bool insert(int x) {
        assert(0<=x && x<n_); if(contains(x)) return false; ++size_;
        for (auto& row:levels_) { auto& word=row[x/64]; bool nonempty=word!=0; word|=std::uint64_t(1)<<(x%64); if(nonempty) break; x/=64; }
        return true;
    }
    bool erase(int x) {
        assert(0<=x && x<n_); if(!contains(x)) return false; --size_;
        for (auto& row:levels_) { auto& word=row[x/64]; word&=~(std::uint64_t(1)<<(x%64)); if(word) break; x/=64; }
        return true;
    }
    int next(int x) const {
        if (x<0) x=0;
        if (x>=n_) return n_;
        for (int h=0;h<int(levels_.size());++h) {
            if(x/64>=int(levels_[h].size())) break;
            auto word=levels_[h][x/64]>>(x%64);
            if(!word) { x=x/64+1; continue; }
            x+=low(word);
            for (int k=h-1;k>=0;--k) x=x*64+low(levels_[k][x]);
            return x<n_?x:n_;
        }
        return n_;
    }
    int prev(int x) const {
        if(x>=n_) x=n_-1;
        if(x<0) return -1;
        for (int h=0;h<int(levels_.size());++h) {
            if(x<0) break;
            auto word=levels_[h][x/64] & (~std::uint64_t(0)>>(63-x%64));
            if(!word) { x=x/64-1; continue; }
            x=x/64*64+high(word);
            for (int k=h-1;k>=0;--k) x=x*64+high(levels_[k][x]);
            return x;
        }
        return -1;
    }
};
} // namespace cp
