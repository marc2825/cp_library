#define PROBLEM "https://judge.yosupo.jp/problem/furthest_pair"
#include <algorithm>
#include <iomanip>
#include <iostream>
#include <vector>
#include "computational_geometry/all.hpp"
namespace e=cp::geometry::exact;namespace f=cp::geometry::floating;
int main(){std::ios::sync_with_stdio(false);std::cin.tie(nullptr);std::cout<<std::fixed<<std::setprecision(15);int t;std::cin>>t;while(t--){int n;std::cin>>n;std::vector<e::Point>p(n);for(auto&a:p)std::cin>>a.x>>a.y;auto h=e::convex_hull(p);if(h.size()==1){std::cout<<"0 1\n";continue;}auto r=e::convex_diameter(h);std::size_t a=0,b=0;while(p[a]!=h[r->first])++a;while(p[b]!=h[r->second])++b;std::cout<<a<<' '<<b<<'\n';}}
