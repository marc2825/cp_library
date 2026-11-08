#include <iostream>
#include <vector>
#include "combinatorial_optimization/general_matching.hpp"
int main(){
    cp::GeneralMatching graph(4);
    graph.add_edge(0,1);graph.add_edge(1,2);graph.add_edge(2,0); // Odd cycle.
    graph.add_edge(2,3);
    auto r=graph.maximum_matching();
    std::cout<<r.cardinality<<'\n';
    for(auto [a,b]:r.pairs)std::cout<<a<<' '<<b<<'\n';
}
