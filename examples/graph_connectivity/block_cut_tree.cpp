#include <iostream>
#include "graph_connectivity/block_cut_tree.hpp"
int main() {
    cp::Graph<int> g(4);g.add_edge(0,1);g.add_edge(1,2);g.add_edge(2,0);g.add_edge(1,3);
    auto r=cp::block_cut_forest(g);std::cout<<r.blocks.size()<<" "<<r.forest.size()<<" "<<r.forest.edge_count()<<"\n";
}
