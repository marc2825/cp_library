#include <iostream>
#include "graph_connectivity/chordal_graph.hpp"
int main() {
    cp::Graph<long long> g(3); g.add_edge(0,1,1); g.add_edge(1,2,2); g.add_edge(0,2,4);
    auto c=cp::chordal_graph(g);
    std::cout << c.is_chordal << ' ' << c.chromatic_number << '\n';
}
