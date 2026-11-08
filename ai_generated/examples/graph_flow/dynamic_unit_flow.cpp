#include <iostream>
#include "graph_flow/dynamic_unit_flow.hpp"
int main() {
    cp::DynamicUnitFlow f(3,0,2);
    int e=f.add_edge(0,1); f.add_edge(1,2);
    std::cout << f.value() << ' ' << f.set_enabled(e,false) << ' ';
    f.add_edge(0,2); std::cout << f.value() << '\n';
}
