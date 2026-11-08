#include <iostream>
#include "graph_connectivity/two_sat.hpp"
int main() {
    cp::TwoSAT s(2);s.force(0,true);s.imply(0,true,1,true);
    auto answer=s.solve(); for(bool x:*answer) { std::cout<<x; } std::cout<<"\n";
}
