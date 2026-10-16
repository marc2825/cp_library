#define PROBLEM "https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=GRL_3_B"
#include <algorithm>
#include <iostream>
#include <utility>
#include <vector>
#include "graph_connectivity/bridges.hpp"
int main(){
    std::ios::sync_with_stdio(false);std::cin.tie(nullptr);
    int n,m;std::cin>>n>>m;cp::Graph<> g(n);
    for(int i=0,u,v;i<m;++i){std::cin>>u>>v;g.add_edge(u,v);}
    std::vector<std::pair<int,int>> answer;
    for(int id:cp::bridges(g)){auto e=g.edges()[id];answer.push_back(std::minmax(e.from,e.to));}
    std::sort(answer.begin(),answer.end());
    for(auto [u,v]:answer)std::cout<<u<<' '<<v<<'\n';
}
