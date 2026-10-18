#include <iostream>
#include "graph_combinatorics/maximal_clique_enumeration.hpp"
int main() {
    cp::Graph<int> g(3); g.add_edge(0,1); g.add_edge(1,2);
    int count=0; cp::enumerate_maximal_cliques(g,[&](const auto&){++count; return true;});
    std::cout << count << '\n';
}
