#define PROBLEM "https://judge.yosupo.jp/problem/assignment"
#include <iostream>
#include <vector>
#include "combinatorial_optimization/hungarian.hpp"
int main(){std::ios::sync_with_stdio(false);std::cin.tie(nullptr);int n;std::cin>>n;std::vector<std::vector<long long>>a(n,std::vector<long long>(n));for(auto&row:a)for(auto&x:row)std::cin>>x;auto r=cp::hungarian(a);std::cout<<r->cost<<'\n';for(auto j:r->row_to_column)std::cout<<j<<' ';std::cout<<'\n';}
