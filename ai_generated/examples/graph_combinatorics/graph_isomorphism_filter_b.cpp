#include <iostream>
#include "graph_combinatorics/graph_isomorphism_filter.hpp"
int main() {
    cp::Graph<int> a(3),b(3); a.add_edge(0,1); a.add_edge(1,2); b.add_edge(2,0); b.add_edge(0,1);
    auto r=cp::graph_isomorphism_filter(a,b);
    std::cout << (r.status!=cp::IsomorphismStatus::different) << '\n';
}
