#include <iostream>
#include "graph_combinatorics/matrix_tree.hpp"
int main() {
    cp::Graph<> g(3);
    g.add_edge(0,1,2); g.add_edge(1,2,3); g.add_edge(0,2,5);
    std::cout<<cp::count_spanning_trees(g)<<'\n';
    std::cout<<cp::count_weighted_spanning_trees(g)<<'\n';
    std::cout<<cp::count_weighted_spanning_trees<2>(g)<<'\n';
}
