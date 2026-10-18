#pragma once
#include <algorithm>
#include <stdexcept>
#include <utility>
#include <vector>
#include "../range_query/fenwick_tree.hpp"

namespace cp {
// 更新座標を事前登録。更新・問い合わせはオンライン。閾値は未登録でも可。
template<class T,class Coord=long long> class CompressedFenwickTree2D {
    std::vector<std::pair<Coord,Coord>> points_; std::vector<Coord> xs_;
    std::vector<std::vector<Coord>> ys_; std::vector<FenwickTree<T>> bits_;
    static void unique(std::vector<Coord>& a) {
        std::sort(a.begin(),a.end());
        a.erase(std::unique(a.begin(),a.end(),[](const Coord& x,const Coord& y){return !(x<y)&&!(y<x);}),a.end());
    }
public:
    explicit CompressedFenwickTree2D(std::vector<std::pair<Coord,Coord>> points):points_(std::move(points)) {
        std::sort(points_.begin(),points_.end());
        for(const auto& p:points_) xs_.push_back(p.first);
        unique(xs_); ys_.resize(xs_.size()+1); bits_.resize(xs_.size()+1);
        for(const auto& p:points_) {
            int x=int(std::lower_bound(xs_.begin(),xs_.end(),p.first)-xs_.begin());
            for(int i=x+1;i<=int(xs_.size());i+=i&-i) ys_[i].push_back(p.second);
        }
        for(int i=1;i<=int(xs_.size());++i) { unique(ys_[i]); bits_[i]=FenwickTree<T>(int(ys_[i].size())); }
    }
    void add(Coord x,Coord y,T value) {
        if(!std::binary_search(points_.begin(),points_.end(),std::make_pair(x,y)))
            throw std::invalid_argument("unregistered update point");
        int p=int(std::lower_bound(xs_.begin(),xs_.end(),x)-xs_.begin());
        for(int i=p+1;i<=int(xs_.size());i+=i&-i)
            bits_[i].add(int(std::lower_bound(ys_[i].begin(),ys_[i].end(),y)-ys_[i].begin()),value);
    }
    // x未満、y未満の点の重み和。
    T sum(Coord x,Coord y) const {
        T result=T(0); int p=int(std::lower_bound(xs_.begin(),xs_.end(),x)-xs_.begin());
        for(int i=p;i>0;i-=i&-i) result+=bits_[i].sum(int(std::lower_bound(ys_[i].begin(),ys_[i].end(),y)-ys_[i].begin()));
        return result;
    }
    T sum(Coord l,Coord d,Coord r,Coord u) const {
        if(!(l<r) || !(d<u)) return T(0);
        return sum(r,u)-sum(l,u)-sum(r,d)+sum(l,d);
    }
    // x以下、y以下。差分矩形の点取得用。座標+1を作らない。
    T sum_inclusive(Coord x,Coord y) const {
        T result=T(0); int p=int(std::upper_bound(xs_.begin(),xs_.end(),x)-xs_.begin());
        for(int i=p;i>0;i-=i&-i) result+=bits_[i].sum(int(std::upper_bound(ys_[i].begin(),ys_[i].end(),y)-ys_[i].begin()));
        return result;
    }
};
} // namespace cp
