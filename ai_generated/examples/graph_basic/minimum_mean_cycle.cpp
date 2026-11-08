#include <iostream>
#include "graph_basic/minimum_mean_cycle.hpp"
int main() {
    cp::Graph<long long> g(3,cp::GraphKind::Directed);
    g.add_edge(0,1,2); g.add_edge(1,2,-4); g.add_edge(2,0,1);
    auto r=cp::minimum_mean_cycle(g);
    if (r) std::cout << cp::optimization_to_string(r->numerator) << ' ' << r->denominator << ' ' << r->edge_ids.size() << '\n';
}
