#include <iostream>
#include "graph_basic/radix_heap_dijkstra.hpp"
int main() {
    cp::Graph<long long> g(3,cp::GraphKind::Directed); g.add_edge(0,1,7); g.add_edge(1,2,3); g.add_edge(0,2,15);
    auto p=cp::radix_heap_dijkstra(g); std::cout<<p.dist[2]<<" ";
    for(int v:p.vertex_path(2)) { std::cout<<v; } std::cout<<"\n";
}
