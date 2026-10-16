#include <iostream>
#include "tree/euler_tour.hpp"
#include "range_query/fenwick_tree.hpp"
int main() {
    cp::Graph<> g(4);
    g.add_edge(0, 1); g.add_edge(1, 2); g.add_edge(1, 3);
    cp::EulerTour<> tour(g);
    cp::FenwickTree<long long> bit(4);
    for (int v = 0; v < 4; ++v) bit.add(tour.tree.tin[v], v + 1);
    auto [l, r] = tour.tree.subtree_interval(1);
    std::cout << bit.sum(l, r) << '\n';
    for (int v : tour.walk) std::cout << v << ' ';
    std::cout << '\n';
}
