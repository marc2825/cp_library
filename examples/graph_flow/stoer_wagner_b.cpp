#include <iostream>
#include "graph_flow/stoer_wagner.hpp"
int main() {
    cp::Graph<long long> g(3); g.add_edge(0,1,1); g.add_edge(1,2,2); g.add_edge(0,2,4);
    auto cut=cp::stoer_wagner(g); std::cout << cut->cost << '\n';
}
