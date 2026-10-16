#define PROBLEM "https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=CGL_4_A"
#include <algorithm>
#include <iomanip>
#include <iostream>
#include <vector>
#include "computational_geometry/all.hpp"
namespace e=cp::geometry::exact;namespace f=cp::geometry::floating;
int main(){std::ios::sync_with_stdio(false);std::cin.tie(nullptr);std::cout<<std::fixed<<std::setprecision(15);int n;std::cin>>n;std::vector<e::Point>p(n);for(auto&a:p)std::cin>>a.x>>a.y;auto h=e::convex_hull(p,true);if(!h.empty())std::rotate(h.begin(),std::min_element(h.begin(),h.end(),[](e::Point a,e::Point b){return a.y!=b.y?a.y<b.y:a.x<b.x;}),h.end());std::cout<<h.size()<<'\n';for(auto a:h)std::cout<<a.x<<' '<<a.y<<'\n';}
