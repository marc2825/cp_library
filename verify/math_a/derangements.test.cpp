#define PROBLEM "https://judge.yosupo.jp/problem/montmort_number_mod"
#include <iostream>
#include <vector>
#include "combinatorics/derangements.hpp"
int main(){int n;long long m;std::cin>>n>>m;auto a=cp::derangements(n,m);for(int i=1;i<=n;++i)std::cout<<a[i]<<(i==n?"\n":" ");}
