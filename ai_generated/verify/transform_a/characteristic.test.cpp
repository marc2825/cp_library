#define PROBLEM "https://judge.yosupo.jp/problem/characteristic_polynomial"
#include <iostream>
#include "linear_algebra/characteristic_polynomial.hpp"
int main(){std::size_t n;std::cin>>n;cp::Matrix<int>a(n,n);for(std::size_t i=0;i<n;++i)for(std::size_t j=0;j<n;++j)std::cin>>a(i,j);for(auto x:cp::characteristic_polynomial_mod(a))std::cout<<x<<' ';std::cout<<'\n';}
