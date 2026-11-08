#include <iostream>
#include "tree/tree_distance_distribution.hpp"
int main() {
    cp::Graph<long long> g(3); g.add_edge(0,1,2); g.add_edge(1,2,3);
    auto c=cp::tree_distance_distribution(g); std::cout << c[1] << ' ' << c[2] << '\n';
}
