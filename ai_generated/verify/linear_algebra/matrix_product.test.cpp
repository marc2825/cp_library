#define PROBLEM "https://judge.yosupo.jp/problem/matrix_product"
#include <iostream>
#include "linear_algebra/matrix_product.hpp"
#include "modint.hpp"
int main() {
    std::ios::sync_with_stdio(false); std::cin.tie(nullptr);
    int n, k, m; std::cin >> n >> k >> m;
    cp::Matrix<TestModInt<998244353>> a(n, k), b(k, m);
    for (int i=0;i<n;++i) for(int j=0;j<k;++j){int x;std::cin>>x;a(i,j)=x;}
    for (int i=0;i<k;++i) for(int j=0;j<m;++j){int x;std::cin>>x;b(i,j)=x;}
    auto c=a*b;
    for(int i=0;i<n;++i){for(int j=0;j<m;++j)std::cout<<(j?" ":"")<<c(i,j).value;std::cout<<'\n';}
}
