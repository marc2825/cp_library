#include <iostream>
#include "graph_combinatorics/euler_circuit_count.hpp"
int main() {
    cp::Graph<int> g(2,cp::GraphKind::Directed);
    for(int i=0;i<2;++i) { g.add_edge(0,1); g.add_edge(1,0); }
    std::cout << cp::count_euler_circuits(g,0) << ' ' << cp::count_euler_circuits(g,0,0) << '\n';
}
