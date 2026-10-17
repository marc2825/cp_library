#include <iostream>
#include "graph_basic/dial_shortest_path.hpp"
int main() {
    cp::Graph<long long> g(3,cp::GraphKind::Directed); g.add_edge(0,1,7); g.add_edge(1,2,3); g.add_edge(0,2,15);
    auto p=cp::dial_shortest_path(g,0,15); std::cout<<p.dist[2]<<"\n";
}
