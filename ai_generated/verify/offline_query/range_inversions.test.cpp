#define PROBLEM "https://judge.yosupo.jp/problem/static_range_inversions_query"
#include <iostream>
#include <utility>
#include <vector>
#include "offline_query/static_range_inversions.hpp"
int main(){
    std::ios::sync_with_stdio(false);std::cin.tie(nullptr);
    int n,q;std::cin>>n>>q;std::vector<long long>a(n);for(auto&x:a)std::cin>>x;
    std::vector<std::pair<int,int>> queries(q);for(auto& [l,r]:queries)std::cin>>l>>r;
    for(auto result:cp::static_range_inversions(a,queries))std::cout<<result<<'\n';
}
