#define PROBLEM "https://judge.yosupo.jp/problem/partition_function"
#include <iostream>
#include <vector>
#include "combinatorics/partition_numbers.hpp"
int main(){int n;std::cin>>n;auto a=cp::partition_numbers(n);for(int i=0;i<=n;++i)std::cout<<a[i]<<(i==n?"\n":" ");}
