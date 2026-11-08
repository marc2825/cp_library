#include <iostream>
#include "union_find/offline_dynamic_connectivity.hpp"
int main() {
    cp::OfflineDynamicConnectivity d(3); d.add_edge(0,1); d.add_query(0,2); d.add_edge(1,2); d.add_query(0,2);
    std::cout << d.solve()[0] << " " << d.solve()[1] << "\n";
}
