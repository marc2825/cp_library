#pragma once
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <random>
#include <vector>
#include "circle_intersection.hpp"

namespace cp { namespace geometry { namespace floating {
namespace circle_detail {
inline bool contains(const Circle&c,Point p,Tolerance t){return t.sign(finite(squared_distance(c.center,p)-finite(c.radius*c.radius)))<=0;}
inline Circle diameter(Point a,Point b){const auto c=a+(b-a)/2;return {c,distance(a,c)};}
inline Circle through_three(Point a,Point b,Point c){
    const auto u=b-a,v=c-a;const auto d=finite(2*cross(u,v));
    if(!d){Circle result=diameter(a,b);for(const auto&candidate:{diameter(a,c),diameter(b,c)})if(candidate.radius>result.radius)result=candidate;return result;}
    const auto uu=norm_squared(u),vv=norm_squared(v);
    const Point offset{finite(finite(finite(v.y*uu)-finite(u.y*vv))/d),finite(finite(finite(u.x*vv)-finite(v.x*uu))/d)};
    const auto center=a+offset;return {center,distance(center,a)};
}
}
// randomized incremental。空はnullopt、固定seed。期待O(n)、最悪O(n^3)。
inline std::optional<Circle> minimum_enclosing_circle(std::vector<Point> points,std::uint64_t seed=20261018,Tolerance t=Tolerance()){
    for(auto p:points)validate(p);
    if(points.empty())return std::nullopt;
    std::mt19937_64 random(seed);std::shuffle(points.begin(),points.end(),random);Circle circle(points[0],0);
    for(std::size_t i=0;i<points.size();++i)if(!circle_detail::contains(circle,points[i],t)){
        circle=Circle(points[i],0);
        for(std::size_t j=0;j<i;++j)if(!circle_detail::contains(circle,points[j],t)){
            circle=circle_detail::diameter(points[i],points[j]);
            for(std::size_t k=0;k<j;++k)if(!circle_detail::contains(circle,points[k],t))circle=circle_detail::through_three(points[i],points[j],points[k]);
        }
    }
    return circle;
}
} } } // namespace cp::geometry::floating
