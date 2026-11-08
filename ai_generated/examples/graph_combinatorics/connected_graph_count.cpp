#include <iostream>
#include "graph_combinatorics/connected_graph_count.hpp"
int main() {
    for(const auto& x:cp::connected_graph_counts(4)) std::cout << x << ' ';
    std::cout << '\n';
}
