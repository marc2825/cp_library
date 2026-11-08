#define PROBLEM "https://judge.yosupo.jp/problem/convolution"
#include <iostream>
#include <vector>
#include "../../convolution/convolution_large.hpp"
int main(){int n,m;std::cin>>n>>m;std::vector<int>a(n),b(m);for(auto&x:a)std::cin>>x;for(auto&x:b)std::cin>>x;auto c=cp::convolution_large(a,b,8);for(std::size_t i=0;i<c.size();++i)std::cout<<(i?" ":"")<<c[i];std::cout<<"\n";}
