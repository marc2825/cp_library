#include <iostream>
#include "graph_combinatorics/disjoint_paths.hpp"
int main() {
    cp::Graph<int> g(4); g.add_edge(0,1); g.add_edge(1,3); g.add_edge(0,2); g.add_edge(2,3);
    std::cout << cp::vertex_disjoint_paths(g,0,3).size() << '\n';
}
