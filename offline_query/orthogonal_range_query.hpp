#pragma once
#include <array>
#include <cstddef>
#include <stdexcept>
#include <vector>
#include "../range_query/fenwick_tree_nd.hpp"
namespace cp {
// 任意次元の静的点集合に対する半開boxの重み和。疎な多次元Fenwickを共有。
template<class T,std::size_t D,class Coordinate=long long> class OrthogonalRangeQuery {
public:
    using Point=std::array<Coordinate,D>;
private:
    FenwickTreeND<T,D,Coordinate> sums_;
    FenwickTreeND<long long,D,Coordinate> counts_;
public:
    OrthogonalRangeQuery(const std::vector<Point>& points,const std::vector<T>& weights)
        : sums_(points),counts_(points) {
        if(points.size()!=weights.size()) throw std::invalid_argument("point/weight size mismatch");
        for(std::size_t i=0;i<points.size();++i) {sums_.add(points[i],weights[i]);counts_.add(points[i],1);}
    }
    explicit OrthogonalRangeQuery(const std::vector<Point>& points={}) : OrthogonalRangeQuery(points,std::vector<T>(points.size(),T(1))) {}
    T sum(const Point& lower,const Point& upper) const {return sums_.sum(lower,upper);}
    long long count(const Point& lower,const Point& upper) const {return counts_.sum(lower,upper);}
};
} // namespace cp
