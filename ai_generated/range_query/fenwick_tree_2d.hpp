#pragma once
#include <cassert>
#include <vector>

namespace cp {
// 密な格子の一点加算・半開長方形和。Tは加法可換群。
template<class T> class FenwickTree2D {
    int h_,w_; std::vector<std::vector<T>> bit_;
public:
    FenwickTree2D(int h,int w):h_(h),w_(w),bit_(h+1,std::vector<T>(w+1,T(0))) { assert(0<=h && h<=100000000 && 0<=w && w<=100000000); }
    void add(int x,int y,T v) {
        assert(0<=x && x<h_ && 0<=y && y<w_);
        for (int i=x+1;i<=h_;i+=i&-i) for (int j=y+1;j<=w_;j+=j&-j) bit_[i][j]+=v;
    }
    T sum(int x,int y) const {
        assert(0<=x && x<=h_ && 0<=y && y<=w_); T v=T(0);
        for (int i=x;i>0;i-=i&-i) for (int j=y;j>0;j-=j&-j) v+=bit_[i][j];
        return v;
    }
    T sum(int left,int down,int right,int up) const {
        assert(0<=left && left<=right && right<=h_ && 0<=down && down<=up && up<=w_);
        return sum(right,up)-sum(left,up)-sum(right,down)+sum(left,down);
    }
};
} // namespace cp
