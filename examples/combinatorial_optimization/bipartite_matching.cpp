#include <iostream>
#include <vector>
#include "combinatorial_optimization/bipartite_matching.hpp"
int main(){
    cp::BipartiteMatching graph(2,3);
    graph.add_edge(0,0);graph.add_edge(0,1);graph.add_edge(1,0);
    auto r=graph.maximum_matching();
    std::cout<<r.cardinality<<'\n';
    for(auto [a,b]:r.pairs)std::cout<<a<<' '<<b<<'\n';
}
