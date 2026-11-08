#include <iostream>
#include "graph_basic/dag_shortest_path.hpp"
int main() {
    cp::Graph<long long> g(3,cp::GraphKind::Directed); g.add_edge(0,1,-2); g.add_edge(1,2,3); g.add_edge(0,2,5);
    auto p=cp::dag_shortest_path(g); std::cout<<p->dist[2]<<"\n";
}
