#include <iostream>
#include "graph_basic/negative_cycle.hpp"
int main() {
    cp::Graph<long long> g(4,cp::GraphKind::Directed);
    g.add_edge(0,1,2); g.add_edge(2,3,-3); g.add_edge(3,2,1);
    auto cycle=cp::find_negative_cycle(g);
    std::cout << bool(cycle) << ' ' << cp::optimization_to_string(cycle->weight)
              << ' ' << bool(cp::find_negative_cycle(g,0)) << '\n';
}
