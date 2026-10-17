#include <iostream>
#include "graph_basic/k_shortest_paths.hpp"
int main() {
    cp::Graph<long long> g(4,cp::GraphKind::Directed); g.add_edge(0,1,1);g.add_edge(1,3,2);g.add_edge(0,2,2);g.add_edge(2,3,2);
    for(auto p:cp::k_shortest_paths(g,0,3,3)) { std::cout<<p.cost<<" "; } std::cout<<"\n";
}
