#pragma once
#include <cstdint>
#include "../common.hpp"
#include "../detail/wide.hpp"
namespace cp { namespace geometry { namespace exact {
struct Point {
    std::int64_t x=0,y=0;
    friend Point operator+(Point a,Point b){return {arithmetic::narrow(arithmetic::add(a.x,b.x)),arithmetic::narrow(arithmetic::add(a.y,b.y))};}
    friend Point operator-(Point a,Point b){return {arithmetic::narrow(arithmetic::sub(a.x,b.x)),arithmetic::narrow(arithmetic::sub(a.y,b.y))};}
    friend Point operator*(Point a,std::int64_t k){return {arithmetic::narrow(arithmetic::mul(a.x,k)),arithmetic::narrow(arithmetic::mul(a.y,k))};}
    friend bool operator==(Point a,Point b){return a.x==b.x&&a.y==b.y;} friend bool operator!=(Point a,Point b){return !(a==b);}
    friend bool operator<(Point a,Point b){return a.x!=b.x?a.x<b.x:a.y<b.y;}
};
using Vector=Point;
inline Wide dot(Point a,Point b){return arithmetic::add(arithmetic::mul(a.x,b.x),arithmetic::mul(a.y,b.y));}
inline Wide cross(Point a,Point b){return arithmetic::sub(arithmetic::mul(a.x,b.y),arithmetic::mul(a.y,b.x));}
inline Wide cross(Point a,Point b,Point c){return arithmetic::sub(arithmetic::mul(arithmetic::sub(b.x,a.x),arithmetic::sub(c.y,a.y)),arithmetic::mul(arithmetic::sub(b.y,a.y),arithmetic::sub(c.x,a.x)));}
inline int orientation(Point a,Point b,Point c){return arithmetic::sign(cross(a,b,c));}
inline Wide norm_squared(Point a){return dot(a,a);}
inline Wide squared_distance(Point a,Point b){const auto x=arithmetic::sub(a.x,b.x),y=arithmetic::sub(a.y,b.y);return arithmetic::add(arithmetic::mul(x,x),arithmetic::mul(y,y));}
struct RationalPoint {
    Rational x,y;
    RationalPoint()=default; RationalPoint(Rational x_,Rational y_):x(x_),y(y_){} explicit RationalPoint(Point p):x(p.x),y(p.y){}
    friend RationalPoint operator+(const RationalPoint&a,const RationalPoint&b){return {a.x+b.x,a.y+b.y};}
    friend RationalPoint operator-(const RationalPoint&a,const RationalPoint&b){return {a.x-b.x,a.y-b.y};}
    friend RationalPoint operator*(const RationalPoint&a,const Rational&k){return {a.x*k,a.y*k};}
    friend bool operator==(const RationalPoint&a,const RationalPoint&b){return a.x==b.x&&a.y==b.y;}
};
inline Rational cross(const RationalPoint&a,const RationalPoint&b){return a.x*b.y-a.y*b.x;}
} } } // namespace cp::geometry::exact
