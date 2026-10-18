#include <iostream>
#include "graph_flow/min_cost_circulation.hpp"
int main() {
    cp::MinCostCirculation<> f(2); f.add_edge(0,1,0,3,-4); f.add_edge(1,0,0,2,1);
    std::cout << cp::optimization_to_string(f.solve()->cost) << '\n';
}
