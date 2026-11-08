#include <iostream>
#include "graph_connectivity/incremental_scc.hpp"
int main() {
    cp::IncrementalSCC s(3); s.add_edge(0,1); s.add_edge(1,2); s.add_edge(2,0);
    std::cout << s.component_count() << ' ' << s.same(0,2) << '\n';
}
