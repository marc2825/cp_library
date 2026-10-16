#define PROBLEM "https://judge.yosupo.jp/problem/longest_increasing_subsequence"
#include <iostream>
#include <vector>
#include "dp_optimization/lis.hpp"
int main() {
    std::ios::sync_with_stdio(false); std::cin.tie(nullptr);
    int n; std::cin>>n; std::vector<long long> a(n); for(auto& x:a)std::cin>>x;
    auto answer=cp::lis(a);std::cout<<answer.length<<'\n';for(int i:answer.indices)std::cout<<i<<' ';std::cout<<'\n';
}
