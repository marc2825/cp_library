#include <iostream>
#include "dp_optimization/monge_shortest_path_k_edges.hpp"
int main() {
    auto r=cp::monge_shortest_path_k_edges(5,2,[](int i,int j){return 1LL*(j-i)*(j-i);});
    std::cout << r->cost << ' ' << r->vertices.size()-1 << '\n';
}
