#include <iostream>
#include "graph_basic/mst_sensitivity.hpp"
int main() {
    cp::Graph<long long> g(3); g.add_edge(0,1,1); g.add_edge(1,2,2); g.add_edge(0,2,4);
    auto result=cp::mst_sensitivity(g);
    std::cout << *result->forced_cost[2] << ' ' << *result->forbidden_cost[1] << '\n';
}
