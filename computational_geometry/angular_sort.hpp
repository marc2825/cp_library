#pragma once
#include <algorithm>
#include <cmath>
#include <vector>
#include "exact/point.hpp"
#include "floating/point.hpp"
namespace cp { namespace geometry {
namespace exact {
// 原点を最初、その後は+xから反時計回り。同じ向きの順番は保持。
inline std::vector<Point> angular_sort(std::vector<Point> p){const auto half=[](Point a){return a.y>0||(a.y==0&&a.x>0)?0:1;};std::stable_sort(p.begin(),p.end(),[&](Point a,Point b){const bool za=a==Point{},zb=b==Point{};if(za||zb)return za&&!zb;if(half(a)!=half(b))return half(a)<half(b);return cross(a,b)>0;});return p;}
}
namespace floating {
// EPSを比較器へ入れず、atan2の有限な数値順を用いる。同じ角度値は安定。
inline std::vector<Point> angular_sort(std::vector<Point> p){for(auto a:p)validate(a);const auto angle=[](Point a){auto t=std::atan2(a.y,a.x);if(t<0)t+=2*std::acos(-1.L);return t;};std::stable_sort(p.begin(),p.end(),[&](Point a,Point b){const bool za=a==Point{},zb=b==Point{};return za||zb?za&&!zb:angle(a)<angle(b);});return p;}
}
} }
