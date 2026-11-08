#define PROBLEM "https://judge.yosupo.jp/problem/system_of_linear_equations"
#include <iostream>
#include <vector>
#include "linear_algebra/gauss_jordan.hpp"
int main() {
    std::ios::sync_with_stdio(false); std::cin.tie(nullptr);
    int n,m;std::cin>>n>>m;cp::Matrix<int>a(n,m);std::vector<int>b(n);
    for(int i=0;i<n;++i)for(int j=0;j<m;++j)std::cin>>a(i,j);
    for(auto&x:b)std::cin>>x;
    auto result=cp::gauss_jordan_mod(a,b);
    if(!result.has_solution()){std::cout<<-1<<'\n';return 0;}
    std::cout<<result.nullspace_basis.size()<<'\n';
    for(auto x:result.particular)std::cout<<x<<' ';
    std::cout<<'\n';
    for(const auto& v:result.nullspace_basis){for(auto x:v)std::cout<<x<<' ';std::cout<<'\n';}
}
