#define PROBLEM "https://judge.yosupo.jp/problem/exp_of_set_power_series"
#include <iostream>
#include <vector>
#include "../../set_function/set_power_series.hpp"
int main(){int n;std::cin>>n;std::vector<int>a(std::size_t(1)<<n);for(auto&x:a)std::cin>>x;auto c=cp::sps_exp(a);for(std::size_t i=0;i<c.size();++i)std::cout<<(i?" ":"")<<c[i];std::cout<<"\n";}
