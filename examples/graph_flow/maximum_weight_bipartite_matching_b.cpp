#include <iostream>
#include "graph_flow/maximum_weight_bipartite_matching.hpp"
int main() {
    auto m=cp::maximum_weight_bipartite_matching(2,2,{{0,0,5},{0,1,8},{1,0,7},{1,1,-1}});
    std::cout << m.weight << '\n';
}
