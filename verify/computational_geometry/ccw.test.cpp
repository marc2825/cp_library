#define PROBLEM "https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=CGL_1_C"
#include <algorithm>
#include <iomanip>
#include <iostream>
#include <vector>
#include "computational_geometry/all.hpp"
namespace e=cp::geometry::exact;namespace f=cp::geometry::floating;
int main(){std::ios::sync_with_stdio(false);std::cin.tie(nullptr);std::cout<<std::fixed<<std::setprecision(15);e::Point a,b;std::cin>>a.x>>a.y>>b.x>>b.y;int q;std::cin>>q;while(q--){e::Point p;std::cin>>p.x>>p.y;auto c=e::ccw(a,b,p);std::cout<<(c==cp::geometry::Ccw::counter_clockwise?"COUNTER_CLOCKWISE":c==cp::geometry::Ccw::clockwise?"CLOCKWISE":c==cp::geometry::Ccw::online_back?"ONLINE_BACK":c==cp::geometry::Ccw::online_front?"ONLINE_FRONT":"ON_SEGMENT")<<'\n';}}
