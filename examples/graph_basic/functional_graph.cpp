#include <iostream>
#include "graph_basic/functional_graph.hpp"
int main() {
    cp::FunctionalGraph f({1,2,1,2});
    std::cout<<f.jump(3,4)<<" "<<*f.min_steps(0,2)<<"\n";
}
