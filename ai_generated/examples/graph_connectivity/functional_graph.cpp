#include <iostream>
#include "graph_connectivity/functional_graph.hpp"
int main() {
    cp::FunctionalGraph f({1,2,1});std::cout<<f.component_count()<<"\n";
}
