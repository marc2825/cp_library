#include <iostream>
#include "graph_flow/general_weighted_matching.hpp"
int main() {
    cp::Graph<long long> g(3);
    g.add_edge(0,1,5); g.add_edge(1,2,7); g.add_edge(2,0,4);
    auto r=cp::general_weighted_matching(g);
    std::cout << cp::optimization_to_string(r.weight) << ' ' << r.edge_ids.size() << '\n';
}
