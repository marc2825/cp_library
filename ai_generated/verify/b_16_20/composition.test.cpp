#define PROBLEM "https://judge.yosupo.jp/problem/composition_of_formal_power_series"
#include <iostream>
#include <vector>
#include "../../fps/composition.hpp"
int main(){int n;std::cin>>n;std::vector<int>a(n),b(n);for(auto&x:a)std::cin>>x;for(auto&x:b)std::cin>>x;auto c=cp::fps_compose(a,b,n);for(std::size_t i=0;i<c.size();++i)std::cout<<(i?" ":"")<<c[i];std::cout<<"\n";}
