#include <iostream>
#include "graph_flow/gomory_hu_tree.hpp"
#include <algorithm>
int main() {
    cp::Graph<long long> g(3);
    g.add_edge(0,1,2); g.add_edge(1,2,3); g.add_edge(0,2,4);
    auto tree=cp::gomory_hu_tree(g);
    std::vector<long long> cuts;
    for (auto e:tree.edges()) cuts.push_back(e.cost);
    std::sort(cuts.begin(),cuts.end());
    for (int i=0;i<int(cuts.size());++i) std::cout << cuts[i] << (i+1==int(cuts.size())?'\n':' ');
}
