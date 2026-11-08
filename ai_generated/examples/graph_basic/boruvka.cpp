#include <iostream>
#include "graph_basic/boruvka.hpp"
int main() {
    auto r=cp::xor_minimum_spanning_tree({0,1,4,5});
    std::cout << cp::optimization_to_string(r.cost) << '\n';
}
