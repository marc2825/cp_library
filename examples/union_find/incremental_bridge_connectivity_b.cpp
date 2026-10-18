#include <iostream>
#include "union_find/incremental_bridge_connectivity.hpp"
int main() {
    cp::IncrementalBridgeConnectivity d(3); d.add_edge(0,1); d.add_edge(1,2); d.add_edge(2,0);
    std::cout << d.bridge_count() << " " << d.two_edge_connected(0,2) << "\n";
}
