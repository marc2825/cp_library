#include <iostream>
#include "graph_connectivity/scc.hpp"
int main() {
    cp::Graph<> g(5,cp::GraphKind::Directed);
    g.add_edge(0,1);g.add_edge(1,0);g.add_edge(1,2);g.add_edge(2,3);g.add_edge(3,2);g.add_edge(3,4);
    auto result=cp::scc(g);
    std::cout<<result.size()<<' '<<result.same(0,1)<<' '<<result.same(0,2)<<'\n';
    for(const auto& group:result.groups){for(int v:group)std::cout<<v<<' ';std::cout<<'\n';}
}
