#define PROBLEM "https://judge.yosupo.jp/problem/segment_add_get_min"
#include <iostream>
#include <tuple>
#include "dp_optimization/li_chao_tree.hpp"
int main(){int n,q;std::cin>>n>>q;std::vector<std::tuple<long long,long long,long long,long long>>initial(n);for(auto &[l,r,a,b]:initial)std::cin>>l>>r>>a>>b;
struct Query{int t;long long l,r,a,b;};std::vector<Query>queries;std::vector<long long>xs;
for(int i=0;i<q;++i){int t;std::cin>>t;if(t==0){long long l,r,a,b;std::cin>>l>>r>>a>>b;queries.push_back({t,l,r,a,b});}else{long long x;std::cin>>x;xs.push_back(x);queries.push_back({t,x,0,0,0});}}
cp::LiChaoTree<>tree(xs);for(auto [l,r,a,b]:initial)tree.add_segment(l,r,a,b);for(auto x:queries)if(x.t==0)tree.add_segment(x.l,x.r,x.a,x.b);else{auto y=tree.query(x.l);if(y)std::cout<<*y<<'\n';else std::cout<<"INFINITY\n";}}
