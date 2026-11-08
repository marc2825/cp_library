#define PROBLEM "https://judge.yosupo.jp/problem/static_range_lis_query"
#include <iostream>
#include <vector>
#include "static_query/static_range_lis.hpp"
int main(){
    std::ios::sync_with_stdio(false);std::cin.tie(nullptr);
    int n,q;std::cin>>n>>q;std::vector<int>a(n);for(int&x:a)std::cin>>x;
    cp::StaticRangeLIS<int> tree(a);
    while(q--){int l,r;std::cin>>l>>r;std::cout<<tree.query(l,r)<<'\n';}
}
