#include <iostream>
#include "graph_combinatorics/min_cost_disjoint_paths.hpp"
int main() {
    cp::Graph<long long> g(4,cp::GraphKind::Directed);
    g.add_edge(0,1,1); g.add_edge(1,3,2); g.add_edge(0,2,2); g.add_edge(2,3,3); g.add_edge(1,2,0);
    auto r=cp::min_cost_edge_disjoint_paths(g,0,3,2);
    auto v=cp::min_cost_vertex_disjoint_paths(g,0,3,2);
    std::cout << r->size() << ' ' << r->cost << ' ' << v->cost << '\n';
}
