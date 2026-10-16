#include <iostream>
#include "graph_connectivity/low_link.hpp"
int main() {
    cp::Graph<> g(5);
    g.add_edge(0,1);g.add_edge(1,2);g.add_edge(2,0);g.add_edge(1,3);g.add_edge(3,4);g.add_edge(3,4);
    auto result=cp::low_link(g);
    for(int id:result.bridge_edges)std::cout<<id<<' ';
    std::cout<<'\n'; // 3
    for(int v:result.articulation)std::cout<<v<<' ';
    std::cout<<'\n'; // 1 3
}
