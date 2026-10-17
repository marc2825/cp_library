#include <iostream>
#include "graph_connectivity/eulerian_trail.hpp"
int main() {
    cp::Graph<int> g(3,cp::GraphKind::Directed);g.add_edge(0,1);g.add_edge(1,0);g.add_edge(0,2);
    auto p=cp::eulerian_trail(g);for(int v:p->vertices) { std::cout<<v; } std::cout<<"\n";
}
