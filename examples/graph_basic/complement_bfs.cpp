#include <iostream>
#include "graph_basic/complement_bfs.hpp"
int main() {
    cp::Graph<int> g(4);g.add_edge(0,1);g.add_edge(1,2);g.add_edge(2,3);
    auto p=cp::complement_bfs(g); std::cout<<p.dist[2]<<" "<<p.dist[1]<<"\n";
}
