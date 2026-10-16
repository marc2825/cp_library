#include <iostream>
#include <vector>
#include "combinatorial_optimization/min_cost_flow.hpp"
int main(){
    cp::MinCostFlow<> network(4);
    network.add_edge(0,1,2,-1);network.add_edge(1,3,2,3);
    network.add_edge(0,2,1,1);network.add_edge(2,3,1,4);
    auto first=network.flow(0,3,1);
    auto rest=network.flow(0,3);
    std::cout<<first.flow<<' '<<first.cost<<'\n'<<rest.flow<<' '<<rest.cost<<'\n';
    network.reset_flow();
    for(auto p:network.slope(0,3))std::cout<<p.flow<<':'<<p.cost<<' ';
    std::cout<<'\n';
}
