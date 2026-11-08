#define PROBLEM "https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=CGL_4_C"
#include <algorithm>
#include <iomanip>
#include <iostream>
#include <vector>
#include "computational_geometry/all.hpp"
namespace e=cp::geometry::exact;namespace f=cp::geometry::floating;
int main(){std::ios::sync_with_stdio(false);std::cin.tie(nullptr);std::cout<<std::fixed<<std::setprecision(15);int n;std::cin>>n;std::vector<f::Point>p(n);for(auto&a:p)std::cin>>a.x>>a.y;int q;std::cin>>q;while(q--){f::Point a,b;std::cin>>a.x>>a.y>>b.x>>b.y;std::cout<<f::polygon_area(f::convex_polygon_cut(p,f::Line(a,b)))<<'\n';}}
