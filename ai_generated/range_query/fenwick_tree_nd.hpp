#pragma once
#include <algorithm>
#include <array>
#include <cassert>
#include <cstddef>
#include <stdexcept>
#include <utility>
#include <vector>
namespace cp {
namespace detail {
template<class T,std::size_t D,class C> class FenwickNDNode {
    std::vector<C> coordinates_;
    std::vector<FenwickNDNode<T,D-1,C>> children_;
    using Point=std::array<C,D>;
    static std::array<C,D-1> tail(const Point& p) {
        std::array<C,D-1> result{}; std::copy(p.begin()+1,p.end(),result.begin()); return result;
    }
public:
    FenwickNDNode()=default;
    explicit FenwickNDNode(const std::vector<Point>& points) {
        for(const auto& p:points) coordinates_.push_back(p[0]);
        std::sort(coordinates_.begin(),coordinates_.end());
        coordinates_.erase(std::unique(coordinates_.begin(),coordinates_.end()),coordinates_.end());
        std::vector<std::vector<std::array<C,D-1>>> buckets(coordinates_.size()+1);
        for(const auto& p:points) {
            std::size_t i=std::lower_bound(coordinates_.begin(),coordinates_.end(),p[0])-coordinates_.begin()+1;
            for(;i<buckets.size();i+=i&-i) buckets[i].push_back(tail(p));
        }
        children_.reserve(buckets.size());
        for(const auto& b:buckets) children_.emplace_back(b);
    }
    void add(const Point& p,const T& x) {
        std::size_t i=std::lower_bound(coordinates_.begin(),coordinates_.end(),p[0])-coordinates_.begin()+1;
        for(;i<children_.size();i+=i&-i) children_[i].add(tail(p),x);
    }
    T prefix(const Point& upper) const {
        T result=T(0);
        std::size_t i=std::lower_bound(coordinates_.begin(),coordinates_.end(),upper[0])-coordinates_.begin();
        for(;i;i-=i&-i) result+=children_[i].prefix(tail(upper));
        return result;
    }
};
template<class T,class C> class FenwickNDNode<T,1,C> {
    std::vector<C> coordinates_; std::vector<T> bit_;
public:
    FenwickNDNode()=default;
    explicit FenwickNDNode(const std::vector<std::array<C,1>>& points) {
        for(const auto& p:points) coordinates_.push_back(p[0]);
        std::sort(coordinates_.begin(),coordinates_.end());
        coordinates_.erase(std::unique(coordinates_.begin(),coordinates_.end()),coordinates_.end());
        bit_.assign(coordinates_.size()+1,T(0));
    }
    void add(const std::array<C,1>& p,const T& x) {
        std::size_t i=std::lower_bound(coordinates_.begin(),coordinates_.end(),p[0])-coordinates_.begin()+1;
        for(;i<bit_.size();i+=i&-i) bit_[i]+=x;
    }
    T prefix(const std::array<C,1>& upper) const {
        T result=T(0);std::size_t i=std::lower_bound(coordinates_.begin(),coordinates_.end(),upper[0])-coordinates_.begin();
        for(;i;i-=i&-i) result+=bit_[i];
        return result;
    }
};
}
// 座標を事前登録する疎な D 次元 Fenwick。T は加法可換群。
template<class T,std::size_t D,class Coordinate=long long> class FenwickTreeND {
    static_assert(D>=1 && D<=10,"dimensions 1..10 required");
public:
    using Point=std::array<Coordinate,D>;
private:
    std::vector<Point> points_;
    detail::FenwickNDNode<T,D,Coordinate> tree_;
public:
    explicit FenwickTreeND(std::vector<Point> points={}) : points_(std::move(points)) {
        std::sort(points_.begin(),points_.end());points_.erase(std::unique(points_.begin(),points_.end()),points_.end());
        tree_=detail::FenwickNDNode<T,D,Coordinate>(points_);
    }
    std::size_t point_count() const { return points_.size(); }
    bool contains(const Point& p) const { return std::binary_search(points_.begin(),points_.end(),p); }
    void add(const Point& p,const T& x) {
        if(!contains(p)) throw std::invalid_argument("unregistered FenwickTreeND point");
        tree_.add(p,x);
    }
    T prefix_sum(const Point& upper) const { return tree_.prefix(upper); }
    T sum(const Point& lower,const Point& upper) const {
        for(std::size_t d=0;d<D;++d) if(!(lower[d]<upper[d])) return T(0);
        T result=T(0);
        for(std::size_t mask=0;mask<(std::size_t(1)<<D);++mask) {
            Point p;bool negative=false;
            for(std::size_t d=0;d<D;++d) {bool low=(mask>>d)&1;p[d]=low?lower[d]:upper[d];negative^=low;}
            T part=tree_.prefix(p);result+=negative?T(0)-part:part;
        }
        return result;
    }
};
} // namespace cp
