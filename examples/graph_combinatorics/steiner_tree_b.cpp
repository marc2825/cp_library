#include <iostream>
#include "graph_combinatorics/steiner_tree.hpp"
int main() {
    cp::Graph<long long> g(3); g.add_edge(0,1,1); g.add_edge(1,2,2); g.add_edge(0,2,4);
    std::cout << cp::steiner_tree(g,{0,2})->cost << '\n';
}
