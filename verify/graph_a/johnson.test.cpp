#define PROBLEM "https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=GRL_1_C"
#include <iostream>
#include "graph_basic/johnson.hpp"
int main(){int n,m;std::cin>>n>>m;cp::Graph<long long>g(n,cp::GraphKind::Directed);for(int i=0;i<m;++i){int u,v;long long w;std::cin>>u>>v>>w;g.add_edge(u,v,w);}auto a=cp::johnson(g);if(!a){std::cout<<"NEGATIVE CYCLE\n";return 0;}for(auto& row:*a){for(int v=0;v<n;++v){if(v)std::cout<<' ';if(row.reachable(v))std::cout<<row.dist[v];else std::cout<<"INF";}std::cout<<'\n';}}
