#include <iostream>
#include "graph_basic/directed_mst.hpp"
int main() {
    cp::Graph<long long> g(3,cp::GraphKind::Directed);
    g.add_edge(0,1,5); g.add_edge(0,2,7); g.add_edge(1,2,1); g.add_edge(2,1,1);
    std::cout << cp::directed_mst(g)->cost << '\n';
}
