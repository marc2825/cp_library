#include <iostream>
#include "graph_basic/difference_constraints.hpp"
int main() {
    cp::DifferenceConstraints<> system(3);
    system.add_constraint(0,1,-2); system.add_constraint(1,2,1); system.add_constraint(0,2,0);
    auto r=system.solve();
    for (int v=0;v<3;++v) std::cout << cp::optimization_to_string(r.assignment[v]) << (v==2?'\n':' ');
    system.add_constraint(2,0,0);
    std::cout << !system.solve().feasible() << '\n';
}
