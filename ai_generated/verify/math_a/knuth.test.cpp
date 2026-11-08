#define PROBLEM "https://atcoder.jp/contests/dp/tasks/dp_n"
#include <iostream>
#include <vector>
#include "dp_optimization/knuth_optimization.hpp"
int main(){int n;std::cin>>n;std::vector<long long> p(n+1);for(int i=0;i<n;++i){long long a;std::cin>>a;p[i+1]=p[i]+a;}std::cout<<cp::knuth_optimization(n,[&](int l,int r){return p[r]-p[l];}).value()<<"\n";}
