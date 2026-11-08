#define PROBLEM "https://judge.yosupo.jp/problem/unionfind"
#include <iostream>
#include "union_find/rollback_disjoint_set_union.hpp"
int main(){int n,q;std::cin>>n>>q;cp::RollbackDisjointSetUnion d(n);while(q--){int t,u,v;std::cin>>t>>u>>v;if(t==0)d.merge(u,v);else std::cout<<d.same(u,v)<<'\n';}}
