#define PROBLEM "https://judge.yosupo.jp/problem/static_range_count_distinct"
#include <iostream>
#include <utility>
#include <vector>
#include "offline_query/static_range_count_distinct.hpp"
int main() {
    std::ios::sync_with_stdio(false); std::cin.tie(nullptr);
    int n,q; std::cin>>n>>q; std::vector<int> a(n); for(auto& x:a) std::cin>>x;
    std::vector<std::pair<int,int>> queries(q); for(auto& query:queries) std::cin>>query.first>>query.second;
    for(int answer:cp::static_range_count_distinct(a,queries)) std::cout<<answer<<'\n';
}
