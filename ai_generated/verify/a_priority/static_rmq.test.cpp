#define PROBLEM "https://judge.yosupo.jp/problem/staticrmq"
#include <iostream>
#include <utility>
#include <vector>
#include "offline_query/offline_rmq.hpp"
int main(){
    int n,q;std::cin>>n>>q;std::vector<int>a(n);for(int& x:a)std::cin>>x;
    std::vector<std::pair<int,int>> queries(q);for(auto& p:queries)std::cin>>p.first>>p.second;
    for(int index:cp::offline_rmq(a,queries))std::cout<<a[index]<<'\n';
}
