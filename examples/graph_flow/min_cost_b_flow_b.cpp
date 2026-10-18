#include <iostream>
#include "graph_flow/min_cost_b_flow.hpp"
int main() {
    cp::MinCostBFlow<> f(2); f.set_balance(0,2); f.set_balance(1,-2); f.add_edge(0,1,0,3,-4);
    auto r=f.solve(); std::cout << cp::optimization_to_string(r->cost) << ' ' << r->flow[0] << '\n';
}
