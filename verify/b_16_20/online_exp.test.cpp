#define PROBLEM "https://judge.yosupo.jp/problem/exp_of_formal_power_series"
#include <iostream>
#include <vector>
#include "../../fps/online_fps.hpp"
int main(){int n;std::cin>>n;cp::OnlineFPSExp<> f(n);std::vector<int> c;for(int i=0;i<n;++i){int a;std::cin>>a;c.push_back(f.append(a));}for(std::size_t i=0;i<c.size();++i)std::cout<<(i?" ":"")<<c[i];std::cout<<"\n";}
