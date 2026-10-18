#include <iostream>
#include "graph_combinatorics/count_four_cycles.hpp"
int main() {
    cp::Graph<int> g(4); for(int u=0;u<4;++u) for(int v=u+1;v<4;++v) g.add_edge(u,v);
    std::cout << cp::count_four_cycles(g) << '\n';
}
