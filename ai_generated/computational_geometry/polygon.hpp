#pragma once
#include <cmath>
#include <utility>
#include <vector>
#include "convex_hull.hpp"
namespace cp { namespace geometry {
namespace exact {
inline Wide signed_area2(const std::vector<Point>&p){Wide sum=0;for(std::size_t i=1;i+1<p.size();++i)sum=arithmetic::add(sum,cross(p[0],p[i],p[i+1]));return sum;}
inline Rational polygon_area(const std::vector<Point>&p){return {arithmetic::abs(signed_area2(p)),2};}
inline Rational signed_area2(const std::vector<RationalPoint>&p){Rational sum;for(std::size_t i=1;i+1<p.size();++i)sum=sum+cross(p[i]-p[0],p[i+1]-p[0]);return sum;}
inline Rational polygon_area(const std::vector<RationalPoint>&p){const auto a=signed_area2(p);return (a.sign()<0?-a:a)/Rational(2);}
inline Location point_in_polygon(const std::vector<Point>&p,Point q){return detail::polygon_contains(p,q,Kernel{});}
inline Location convex_polygon_contains(const std::vector<Point>&hull,Point q){return detail::convex_contains(hull,q,Kernel{});}
class ConvexPolygon {
    std::vector<Point> points_;
public:
    explicit ConvexPolygon(std::vector<Point> points={}):points_(convex_hull(std::move(points))){}
    const std::vector<Point>& vertices()const{return points_;}
    Location contains(Point p)const{return convex_polygon_contains(points_,p);}
};
}
namespace floating {
inline long double signed_area2(const std::vector<Point>&p){for(auto a:p)validate(a);long double sum=0;for(std::size_t i=1;i+1<p.size();++i)sum=finite(sum+cross(p[0],p[i],p[i+1]));return sum;}
inline long double polygon_area(const std::vector<Point>&p){return std::abs(signed_area2(p))/2;}
inline Location point_in_polygon(const std::vector<Point>&p,Point q,Tolerance t=Tolerance()){return detail::polygon_contains(p,q,Kernel(t));}
inline Location convex_polygon_contains(const std::vector<Point>&hull,Point q,Tolerance t=Tolerance()){return detail::convex_contains(hull,q,Kernel(t));}
class ConvexPolygon {
    std::vector<Point> points_;Tolerance tolerance_;
public:
    explicit ConvexPolygon(std::vector<Point> points={},Tolerance t=Tolerance()):points_(convex_hull(std::move(points),false,t)),tolerance_(t){}
    const std::vector<Point>& vertices()const{return points_;}
    Location contains(Point p)const{return convex_polygon_contains(points_,p,tolerance_);}
};
}
} }
