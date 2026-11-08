#define PROBLEM "https://judge.yosupo.jp/problem/closest_pair"
#include <algorithm>
#include <iomanip>
#include <iostream>
#include <vector>
#include "computational_geometry/all.hpp"
namespace e=cp::geometry::exact;namespace f=cp::geometry::floating;
int main(){std::ios::sync_with_stdio(false);std::cin.tie(nullptr);std::cout<<std::fixed<<std::setprecision(15);int t;std::cin>>t;while(t--){int n;std::cin>>n;std::vector<e::Point>p(n);for(auto&a:p)std::cin>>a.x>>a.y;auto r=e::closest_pair(p);std::cout<<r->first<<' '<<r->second<<'\n';}}
