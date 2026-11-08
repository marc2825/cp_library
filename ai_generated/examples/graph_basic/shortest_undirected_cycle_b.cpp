#include <iostream>
#include "graph_basic/shortest_undirected_cycle.hpp"
int main() {
    cp::Graph<long long> g(3);
    g.add_edge(0,1,2); g.add_edge(1,2,3); g.add_edge(2,0,4);
    std::cout << cp::shortest_undirected_cycle(g)->cost << '\n';
}
