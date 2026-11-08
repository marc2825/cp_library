#include <iostream>
#include "graph_basic/second_best_mst.hpp"
int main() {
    cp::Graph<long long> g(3); g.add_edge(0,1,1); g.add_edge(1,2,2); g.add_edge(0,2,4);
    std::cout << cp::second_best_spanning_tree(g,true)->cost << '\n';
}
