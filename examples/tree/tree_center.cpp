#include <iostream>
#include "../../tree/tree_center.hpp"
int main() {
    cp::Graph<long long> g(3);
    g.add_edge(0, 1, 2); g.add_edge(1, 2, 5);
    auto c = cp::tree_center(g);
    std::cout << c.vertices[0] << '\n';
    auto w = cp::weighted_tree_center(g);
    std::cout << w.vertices[0] << ' ' << w.vertex_radius << '\n';
    std::cout << w.edge_id << ' ' << w.offset_twice << ' ' << w.radius_twice << '\n';
}
