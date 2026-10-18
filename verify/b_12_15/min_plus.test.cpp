#define PROBLEM "https://judge.yosupo.jp/problem/min_plus_convolution_concave_arbitrary"
#include <iostream>
#include <vector>
#include "dp_optimization/min_plus_convolution.hpp"
int main(){int n,m;if(!(std::cin>>n>>m))return 0;std::vector<long long>a(n),b(m);for(auto& x:a)std::cin>>x;for(auto& x:b)std::cin>>x;auto c=cp::min_plus_convolution_concave_arbitrary(a,b);for(std::size_t i=0;i<c.size();++i)std::cout<<c[i]<<(i+1==c.size()?'\n':' ');}
