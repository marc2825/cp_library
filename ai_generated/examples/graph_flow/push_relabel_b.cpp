#include <iostream>
#include "graph_flow/push_relabel.hpp"
int main() {
    cp::PushRelabel<> f(3); f.add_edge(0,1,5); f.add_edge(1,2,3); f.add_edge(0,2,1);
    std::cout << f.solve(0,2).value << '\n';
}
