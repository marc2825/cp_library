#define PROBLEM "https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=CGL_1_A"
#include <algorithm>
#include <iomanip>
#include <iostream>
#include <vector>
#include "computational_geometry/all.hpp"
namespace e=cp::geometry::exact;namespace f=cp::geometry::floating;
int main(){std::ios::sync_with_stdio(false);std::cin.tie(nullptr);std::cout<<std::fixed<<std::setprecision(15);f::Point a,b;std::cin>>a.x>>a.y>>b.x>>b.y;f::Line l(a,b);int q;std::cin>>q;while(q--){f::Point p;std::cin>>p.x>>p.y;auto r=f::projection(l,p);std::cout<<r.x<<' '<<r.y<<'\n';}}
