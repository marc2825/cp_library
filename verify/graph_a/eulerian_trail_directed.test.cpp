#define PROBLEM "https://judge.yosupo.jp/problem/eulerian_trail_directed"
#include <iostream>
#include "graph_connectivity/eulerian_trail.hpp"
int main(){int cases;std::cin>>cases;while(cases--){int n,m;std::cin>>n>>m;cp::Graph<int>g(n,cp::GraphKind::Directed);for(int i=0;i<m;++i){int u,v;std::cin>>u>>v;g.add_edge(u,v);}auto p=cp::eulerian_trail(g);if(!p){std::cout<<"No\n";continue;}std::cout<<"Yes\n";for(int i=0;i<int(p->vertices.size());++i){if(i)std::cout<<' ';std::cout<<p->vertices[i];}std::cout<<'\n';for(int i=0;i<int(p->edge_ids.size());++i){if(i)std::cout<<' ';std::cout<<p->edge_ids[i];}std::cout<<'\n';}}
