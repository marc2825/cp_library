#include <iostream>
#include <vector>
#include "../../linear_algebra/matrix_tree.hpp"

int main() {
    cp::Graph<int> g(3);
    g.add_edge(0, 1, 2); g.add_edge(1, 2, 3); g.add_edge(0, 2, 5);
    std::cout << cp::count_weighted_spanning_trees(g) << "\n";
}
