#define PROBLEM "https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=DPL_3_B"
#include <iostream>
#include <vector>
#include "dp_optimization/largest_rectangle.hpp"
int main(){std::ios::sync_with_stdio(false);std::cin.tie(nullptr);int h,w;std::cin>>h>>w;std::vector<std::vector<bool>>blocked(h,std::vector<bool>(w));for(auto&row:blocked)for(int j=0;j<w;++j){int x;std::cin>>x;row[j]=x;}std::cout<<cp::largest_rectangle_grid(blocked).area<<'\n';}
