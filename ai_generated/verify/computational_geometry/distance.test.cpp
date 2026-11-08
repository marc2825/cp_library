#define PROBLEM "https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=CGL_2_D"
#include <algorithm>
#include <iomanip>
#include <iostream>
#include <vector>
#include "computational_geometry/all.hpp"
namespace e=cp::geometry::exact;namespace f=cp::geometry::floating;
int main(){std::ios::sync_with_stdio(false);std::cin.tie(nullptr);std::cout<<std::fixed<<std::setprecision(15);int q;std::cin>>q;while(q--){f::Segment a,b;std::cin>>a.a.x>>a.a.y>>a.b.x>>a.b.y>>b.a.x>>b.a.y>>b.b.x>>b.b.y;std::cout<<f::distance(a,b)<<'\n';}}
