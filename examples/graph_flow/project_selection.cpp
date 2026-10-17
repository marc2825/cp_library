#include <iostream>
#include "graph_flow/project_selection.hpp"
int main() {
    cp::ProjectSelection<> p(2);p.add_profit(0,7);p.add_profit(1,-2);p.imply(0,1);
    auto a=p.solve();std::cout<<a->cost<<" ";for(bool b:a->selected)std::cout<<b;std::cout<<"\n";
}
