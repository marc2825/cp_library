#include <iostream>
#include "graph_flow/dinic.hpp"
int main() {
    cp::Dinic<long long> network(4);
    network.add_edge(0, 1, 3);
    network.add_edge(0, 2, 2);
    network.add_edge(1, 2, 1);
    network.add_edge(1, 3, 2);
    network.add_edge(2, 3, 3);
    std::cout << network.max_flow(0, 3, 2) << '\n'; // Additional 2.
    std::cout << network.max_flow(0, 3) << '\n';    // Additional 3.
    std::cout << network.max_flow(0, 3) << '\n';    // Additional 0.
    for (auto e : network.edges()) std::cout << e.flow << '/' << e.capacity << ' ';
    std::cout << '\n';
}
