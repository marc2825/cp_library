#define PROBLEM "https://onlinejudge.u-aizu.ac.jp/problems/CGL_7_E"
#include <iomanip>
#include <iostream>
#include "../../computational_geometry/circle_intersection.hpp"
int main(){namespace g=cp::geometry::floating;long double x,y,r,a,b,s;std::cin>>x>>y>>r>>a>>b>>s;const auto result=g::circle_circle_intersection(g::Circle({x,y},r),g::Circle({a,b},s));const auto first=result.points.front(),last=result.points.back();std::cout<<std::fixed<<std::setprecision(18)<<first.x<<' '<<first.y<<' '<<last.x<<' '<<last.y<<'\n';}
