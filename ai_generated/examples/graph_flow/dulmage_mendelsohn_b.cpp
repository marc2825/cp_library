#include <iostream>
#include "graph_flow/dulmage_mendelsohn.hpp"
int main() {
    auto d=cp::dulmage_mendelsohn(2,2,{{0,0},{0,1},{1,1}});
    for(bool x:d.allowed_edge) { std::cout << x; }
    std::cout << '\n';
}
