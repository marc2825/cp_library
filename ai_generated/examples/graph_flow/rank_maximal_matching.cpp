#include <iostream>
#include "graph_flow/rank_maximal_matching.hpp"
int main() {
    auto r=cp::rank_maximal_matching(2,2,{{0,0,1},{0,1,2},{1,0,2}});
    std::cout << r.profile[0] << ' ' << r.profile[1] << '\n';
}
