#include <iostream>
#include "graph_connectivity/dag_reachability.hpp"
int main() {
    cp::Graph<long long> g(3,cp::GraphKind::Directed);
    g.add_edge(0,1); g.add_edge(1,2);
    cp::DAGReachability r(g); std::cout << r.reachable(0,2) << ' ' << r.reachable(2,0) << '\n';
}
