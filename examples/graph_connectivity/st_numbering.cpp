#include <iostream>
#include "graph_connectivity/st_numbering.hpp"
int main() {
    cp::Graph<int> g(4);
    g.add_edge(0,1); g.add_edge(1,2); g.add_edge(2,3); g.add_edge(3,0);
    auto r=cp::st_numbering(g,0,2);
    if (r) std::cout << r->order.front() << ' ' << r->order.back() << ' ' << r->oriented_edges.size() << '\n';
}
