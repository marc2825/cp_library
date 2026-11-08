#define PROBLEM "https://judge.yosupo.jp/problem/range_kth_smallest"
#include <iostream>
#include "ordered_structure/persistent_order_statistics.hpp"
int main() { int n,q; std::cin>>n>>q; std::vector<int>a(n); for(int &x:a)std::cin>>x; cp::PersistentOrderStatistics<int>s(a); while(q--){int l,r,k;std::cin>>l>>r>>k;std::cout<<*s.kth_smallest(l,r,k)<<'\n';} }
