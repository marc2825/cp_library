#include <iostream>
#include "graph_basic/johnson.hpp"
int main() {
    cp::Graph<long long> g(3,cp::GraphKind::Directed); g.add_edge(0,1,-2); g.add_edge(1,2,3); g.add_edge(0,2,5);
    auto all=cp::johnson(g); std::cout<<(*all)[0].dist[2]<<" "<<(*all)[2].reachable(0)<<"\n";
}
