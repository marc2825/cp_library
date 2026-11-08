#include <iostream>
#include "graph_combinatorics/dag_minimum_path_cover.hpp"
int main() {
    cp::Graph<long long> g(3,cp::GraphKind::Directed);
    g.add_edge(0,1); g.add_edge(1,2);
    std::cout << cp::dag_minimum_path_cover(g).size() << '\n';
}
