#include <iostream>
#include "graph_basic/manhattan_mst.hpp"
int main() {
    auto mst=cp::manhattan_mst({{0,0},{2,0},{2,3}});
    std::cout << mst.cost << '\n';
}
