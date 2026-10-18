#pragma once
#include <algorithm>
#include <vector>
#include "convex_cut.hpp"
namespace cp { namespace geometry {
namespace exact {
// 指定した閉矩形の中で半平面の共通部分を求める。人工的な無限大を使わない。
// 各Lineの左側（境界込み）。空/点/線分も返す。O(h^2)。
inline std::vector<RationalPoint> half_plane_intersection(const std::vector<Line>&halfplanes,Point lower,Point upper){
    if(upper.x<lower.x||upper.y<lower.y)throw std::invalid_argument("reversed half-plane bounding box");
    std::vector<RationalPoint> polygon;for(Point p:{lower,Point{upper.x,lower.y},upper,Point{lower.x,upper.y}})if(polygon.empty()||!(polygon.back()==RationalPoint(p)))polygon.emplace_back(p);
    if(polygon.size()>1&&polygon.front()==polygon.back())polygon.pop_back();
    for(const auto&line:halfplanes){validate(line);polygon=convex_polygon_cut(polygon,line);}return polygon;
}
}
namespace floating {
inline std::vector<Point> half_plane_intersection(const std::vector<Line>&halfplanes,Point lower,Point upper,Tolerance t=Tolerance()){
    validate(lower);validate(upper);if(upper.x<lower.x||upper.y<lower.y)throw std::invalid_argument("reversed half-plane bounding box");
    std::vector<Point> polygon;for(Point p:{lower,Point{upper.x,lower.y},upper,Point{lower.x,upper.y}})if(polygon.empty()||polygon.back()!=p)polygon.push_back(p);
    if(polygon.size()>1&&polygon.front()==polygon.back())polygon.pop_back();
    for(const auto&line:halfplanes){validate(line);polygon=convex_polygon_cut(polygon,line,t);}return polygon;
}
}
} }
