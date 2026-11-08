#define PROBLEM "https://judge.yosupo.jp/problem/pow_of_matrix"
#include <cstdint>
#include <iostream>
#include "linear_algebra/matrix_power.hpp"
#include "modint.hpp"
int main() {
    std::ios::sync_with_stdio(false); std::cin.tie(nullptr);
    int n; std::uint64_t exponent; std::cin >> n >> exponent;
    cp::Matrix<TestModInt<998244353>> a(n,n);
    for(int i=0;i<n;++i)for(int j=0;j<n;++j){int x;std::cin>>x;a(i,j)=x;}
    auto c=cp::matrix_power(a,exponent);
    for(int i=0;i<n;++i){for(int j=0;j<n;++j)std::cout<<(j?" ":"")<<c(i,j).value;std::cout<<'\n';}
}
