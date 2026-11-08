#include <iostream>
#include "graph_connectivity/condensation.hpp"
int main() {
    cp::Graph<> g(3,cp::GraphKind::Directed);
    g.add_edge(0,1);g.add_edge(1,0);g.add_edge(0,2);g.add_edge(1,2);
    auto result=cp::condensation_graph(g);
    std::cout<<result.dag.size()<<' '<<result.dag.edge_count()<<'\n'; // 2 1
    for(auto e:result.dag.edges())std::cout<<e.from<<' '<<e.to<<' '<<result.original_edge[e.idx]<<'\n';
    std::cout<<cp::condensation_graph(g,false).dag.edge_count()<<'\n'; // 2
}
