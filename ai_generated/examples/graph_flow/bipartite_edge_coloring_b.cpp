#include <iostream>
#include "graph_flow/bipartite_edge_coloring.hpp"
int main() {
    auto c=cp::bipartite_edge_coloring(2,2,{{0,0},{0,1},{1,0},{1,1}});
    std::cout << c.color_count << '\n';
}
