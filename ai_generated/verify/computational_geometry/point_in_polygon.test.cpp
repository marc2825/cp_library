#define PROBLEM "https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=CGL_3_C"
#include <algorithm>
#include <iomanip>
#include <iostream>
#include <vector>
#include "computational_geometry/all.hpp"
namespace e=cp::geometry::exact;namespace f=cp::geometry::floating;
int main(){std::ios::sync_with_stdio(false);std::cin.tie(nullptr);std::cout<<std::fixed<<std::setprecision(15);int n;std::cin>>n;std::vector<e::Point>p(n);for(auto&a:p)std::cin>>a.x>>a.y;int q;std::cin>>q;while(q--){e::Point a;std::cin>>a.x>>a.y;std::cout<<static_cast<int>(e::point_in_polygon(p,a))<<'\n';}}
