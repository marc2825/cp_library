#define PROBLEM "https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=CGL_2_C"
#include <algorithm>
#include <iomanip>
#include <iostream>
#include <vector>
#include "computational_geometry/all.hpp"
namespace e=cp::geometry::exact;namespace f=cp::geometry::floating;
int main(){std::ios::sync_with_stdio(false);std::cin.tie(nullptr);std::cout<<std::fixed<<std::setprecision(15);int q;std::cin>>q;while(q--){f::Point a,b,c,d;std::cin>>a.x>>a.y>>b.x>>b.y>>c.x>>c.y>>d.x>>d.y;auto r=f::line_intersection(f::Line(a,b),f::Line(c,d));std::cout<<r.point->x<<' '<<r.point->y<<'\n';}}
