#include <iostream>
#include "graph_connectivity/cograph_canonical_form.hpp"
int main() {
    cp::Graph<int> g(3); g.add_edge(0,1); g.add_edge(1,2);
    auto r=cp::cograph_canonical_form(g);
    std::cout << r->code << '\n';
}
