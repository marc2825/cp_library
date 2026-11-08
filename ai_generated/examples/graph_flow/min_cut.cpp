#include <iostream>
#include "graph_flow/min_cut.hpp"
int main() {
    cp::Dinic<> network(4);
    network.add_edge(0, 1, 3);
    network.add_edge(0, 2, 2);
    network.add_edge(1, 2, 1);
    network.add_edge(1, 3, 2);
    network.add_edge(2, 3, 3);
    const auto cut = cp::minimum_cut(network, 0, 3); // Also completes max flow.
    std::cout << cut.capacity << '\n';
    for (int v = 0; v < network.size(); ++v) if (cut.source_side[v]) std::cout << v << ' ';
    std::cout << '\n';
    for (int id : cut.edge_ids) std::cout << id << ' ';
    std::cout << '\n';
}
