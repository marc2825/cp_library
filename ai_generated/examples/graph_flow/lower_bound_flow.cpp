#include <iostream>
#include "graph_flow/lower_bound_flow.hpp"
int main() {
    cp::LowerBoundFlow<> f(3);f.add_edge(0,1,1,4);f.add_edge(1,2,1,3);f.add_edge(0,2,0,2);
    auto a=f.maximum_flow(0,2);std::cout<<a->value;for(auto x:a->flow)std::cout<<" "<<x;std::cout<<"\n";
}
