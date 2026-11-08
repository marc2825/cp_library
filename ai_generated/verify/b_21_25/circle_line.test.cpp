#define PROBLEM "https://onlinejudge.u-aizu.ac.jp/problems/CGL_7_D"
#include <iomanip>
#include <iostream>
#include "../../computational_geometry/circle_intersection.hpp"
int main(){namespace g=cp::geometry::floating;long double x,y,r;int q;std::cin>>x>>y>>r>>q;g::Circle circle({x,y},r);std::cout<<std::fixed<<std::setprecision(18);for(int i=0;i<q;++i){long double a,b,c,d;std::cin>>a>>b>>c>>d;const auto result=g::circle_line_intersection(circle,g::Line({a,b},{c,d}));const auto first=result.points.front(),last=result.points.back();std::cout<<first.x<<' '<<first.y<<' '<<last.x<<' '<<last.y<<'\n';}}
