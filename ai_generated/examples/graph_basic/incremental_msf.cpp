#include <iostream>
#include "graph_basic/incremental_msf.hpp"
int main() {
    cp::IncrementalMSF<> forest(4);
    for (auto e:std::vector<cp::IncrementalMSF<>::EdgeInfo>{{0,1,5},{1,2,4},{0,2,1},{2,3,2}}) {
        forest.add_edge(e.from,e.to,e.cost);
        std::cout << cp::optimization_to_string(forest.cost()) << '\n';
    }
    std::cout << forest.selected_edges().size() << '\n';
}
