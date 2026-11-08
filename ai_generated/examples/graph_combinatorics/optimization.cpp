#include <iostream>
#include "graph_combinatorics/maximum_clique.hpp"
#include "graph_combinatorics/maximum_independent_set.hpp"
int main() {
    cp::Graph<> g(5);
    for(int v=0;v<5;++v) g.add_edge(v,(v+1)%5);
    std::cout<<cp::maximum_clique(g).size()<<' '<<cp::maximum_independent_set(g).size()<<'\n';
}
