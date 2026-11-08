#include <iostream>
#include "graph_basic/k_shortest_walks.hpp"
int main() {
    cp::Graph<long long> g(2,cp::GraphKind::Directed);g.add_edge(0,1,2);g.add_edge(1,1,1);
    auto a=cp::k_shortest_walks(g,0,1,3);for(auto c:a.costs) std::cout<<c<<" ";
    std::cout<<"\n"<<a.path(2).edge_ids.size()<<"\n";
}
