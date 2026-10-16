#include <iostream>
#include "offline_query/offline_dynamic_connectivity.hpp"
int main() {
    cp::OfflineDynamicConnectivity g(3); g.add_edge(0, 1); g.add_edge(1, 2);
    g.add_query(0, 2); g.remove_edge(1, 2); g.add_query(0, 2);
    for (bool b : g.solve()) std::cout << b << ' ';
    std::cout << '\n';
}
