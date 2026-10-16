#define PROBLEM "https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=DPL_3_C"
#include <iostream>
#include <vector>
#include "dp_optimization/largest_rectangle.hpp"
int main(){std::ios::sync_with_stdio(false);std::cin.tie(nullptr);int n;std::cin>>n;std::vector<long long>h(n);for(auto&x:h)std::cin>>x;std::cout<<cp::largest_rectangle_histogram(h).area<<'\n';}
