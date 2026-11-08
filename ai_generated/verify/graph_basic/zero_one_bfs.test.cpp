#define PROBLEM "https://atcoder.jp/contests/abc077/tasks/arc084_b"
#include <iostream>
#include "graph_basic/all.hpp"
int main() {
    std::ios::sync_with_stdio(false); std::cin.tie(nullptr);
    int k; std::cin>>k; cp::Graph<> g(k,cp::GraphKind::Directed);
    for(int i=0;i<k;++i){g.add_edge(i,i*10%k,0);g.add_edge(i,(i+1)%k,1);}
    std::cout<<cp::zero_one_bfs(g,1%k).dist[0]+1<<'\n';
}
