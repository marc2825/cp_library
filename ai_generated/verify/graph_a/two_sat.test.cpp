#define PROBLEM "https://judge.yosupo.jp/problem/two_sat"
#include <iostream>
#include <string>
#include "graph_connectivity/two_sat.hpp"
int main(){std::string p,cnf;int n,m;std::cin>>p>>cnf>>n>>m;cp::TwoSAT sat(n);for(int i=0;i<m;++i){int a,b,z;std::cin>>a>>b>>z;sat.add_clause((a>0?a:-a)-1,a>0,(b>0?b:-b)-1,b>0);}auto result=sat.solve();if(!result){std::cout<<"s UNSATISFIABLE\n";return 0;}std::cout<<"s SATISFIABLE\nv ";for(int i=0;i<n;++i)std::cout<<((*result)[i]?i+1:-(i+1))<<' ';std::cout<<"0\n";}
