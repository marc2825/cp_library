#include <iostream>
#include "graph_flow/bipartite_vertex_cover.hpp"
int main() {
    auto r=cp::bipartite_vertex_cover(3,3,{{0,0},{1,0},{2,1}});
    std::cout << r.matching.cardinality << ' ' << r.left_cover.size()+r.right_cover.size()
              << ' ' << r.left_independent.size()+r.right_independent.size() << '\n';
}
