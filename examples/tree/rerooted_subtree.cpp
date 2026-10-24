#include <iostream>
#include "../../tree/rerooted_subtree.hpp"
#include "../../range_query/fenwick_tree.hpp"
int main() {
    cp::Graph<int> g(5);
    g.add_edge(0, 1); g.add_edge(0, 2); g.add_edge(1, 3); g.add_edge(1, 4);
    cp::HeavyLightDecomposition<int> hld(g);
    cp::FenwickTree<long long> bit(5);
    for (int v = 0; v < 5; ++v) bit.add(hld.pos[v], v + 1);
    long long sum = 0;
    for (auto [l, r] : cp::rerooted_subtree_intervals(hld, 3, 1)) sum += bit.sum(l, r);
    std::cout << sum << '\n';
}
