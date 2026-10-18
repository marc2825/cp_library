#define PROBLEM "https://judge.yosupo.jp/problem/min_cost_b_flow"
#include <iostream>
#include "graph_flow/min_cost_b_flow.hpp"
int main(){int n,m;if(!(std::cin>>n>>m))return 0;cp::MinCostBFlow<> f(n);for(int v=0;v<n;++v){long long b;std::cin>>b;f.set_balance(v,b);}for(int i=0;i<m;++i){int u,v;long long l,r,c;std::cin>>u>>v>>l>>r>>c;f.add_edge(u,v,l,r,c);}auto a=f.solve();if(!a){std::cout<<"infeasible\n";return 0;}std::cout<<cp::optimization_to_string(a->cost)<<'\n';for(auto p:a->potential)std::cout<<cp::optimization_to_string(p)<<'\n';for(auto x:a->flow)std::cout<<x<<'\n';}
