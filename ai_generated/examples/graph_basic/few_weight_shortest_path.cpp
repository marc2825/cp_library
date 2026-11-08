#include <iostream>
#include "graph_basic/few_weight_shortest_path.hpp"
int main() {
    cp::Graph<long long> g(3,cp::GraphKind::Directed);
    g.add_edge(0,1,2); g.add_edge(1,2,2); g.add_edge(0,2,100);
    auto r=cp::few_weight_shortest_path(g,0);
    for(auto d:r.dist) std::cout << d << ' ';
    std::cout << '\n';
}
