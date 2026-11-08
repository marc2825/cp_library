#define PROBLEM "https://judge.yosupo.jp/problem/sort_points_by_argument"
#include <algorithm>
#include <iomanip>
#include <iostream>
#include <vector>
#include "computational_geometry/all.hpp"
namespace e=cp::geometry::exact;namespace f=cp::geometry::floating;
int main(){std::ios::sync_with_stdio(false);std::cin.tie(nullptr);std::cout<<std::fixed<<std::setprecision(15);int n;std::cin>>n;std::vector<e::Point>p(n);for(auto&a:p)std::cin>>a.x>>a.y;p=e::angular_sort(p);std::stable_partition(p.begin(),p.end(),[](e::Point a){return a.y<0;});for(auto a:p)std::cout<<a.x<<' '<<a.y<<'\n';}
