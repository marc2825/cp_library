#pragma once
#include <algorithm>
#include <cmath>
#include <vector>
#include "floating/primitives.hpp"

namespace cp { namespace geometry { namespace floating {
struct Circle {Point center;long double radius=0;Circle(Point c={},long double r=0):center(c),radius(r){validate(center);input(r);if(r<0)throw std::invalid_argument("circle radius must be nonnegative");}};
inline void validate(const Circle&c){validate(c.center);input(c.radius);if(c.radius<0)throw std::invalid_argument("circle radius must be nonnegative");}
struct CircleIntersections {bool coincident=false;std::vector<Point> points;};
// 同一の正半径円ならcoincident=true。点円の同一点は1交点。
inline CircleIntersections circle_circle_intersection(const Circle&a,const Circle&b,Tolerance t=Tolerance()){
    validate(a);validate(b);const auto delta=b.center-a.center;const auto d=distance(a.center,b.center);
    if(!t.sign(d)){
        if(t.sign(finite(a.radius-b.radius)))return {};
        if(!a.radius&&!b.radius)return {false,{a.center}};
        return {true,{}};
    }
    if(t.sign(finite(d-finite(a.radius+b.radius)))>0||t.sign(finite(d-std::abs(finite(a.radius-b.radius))))<0)return {};
    const auto along=finite(finite(finite(a.radius*a.radius)-finite(b.radius*b.radius)+finite(d*d))/finite(2*d));
    const auto height2=finite(finite(a.radius*a.radius)-finite(along*along));
    if(t.sign(height2)<0)return {};
    const auto unit=delta/d,center=a.center+unit*along;
    if(t.sign(height2)==0)return {false,{center}};
    const auto offset=Point{-unit.y,unit.x}*finite(std::sqrt(height2));
    std::vector<Point> points{center-offset,center+offset};std::sort(points.begin(),points.end());return {false,points};
}
inline CircleIntersections circle_line_intersection(const Circle&circle,const Line&line,Tolerance t=Tolerance()){
    validate(circle);validate(line);const auto center=projection(line,circle.center);
    const auto height2=finite(finite(circle.radius*circle.radius)-squared_distance(center,circle.center));
    if(t.sign(height2)<0)return {};
    if(t.sign(height2)==0)return {false,{center}};
    const auto direction=line.b-line.a;const auto norm=finite(std::sqrt(norm_squared(direction)));if(!norm)throw std::domain_error("line direction underflow");
    const auto offset=direction*finite(std::sqrt(height2)/norm);std::vector<Point> points{center-offset,center+offset};std::sort(points.begin(),points.end());return {false,points};
}
} } } // namespace cp::geometry::floating
