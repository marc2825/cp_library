#include <iostream>
#include "graph_connectivity/connected_components.hpp"
#include "graph_connectivity/bipartite.hpp"
int main() {
    cp::Graph<> g(5);g.add_edge(0,1);g.add_edge(1,2);g.add_edge(3,4);
    auto components=cp::connected_components(g);
    std::cout<<components.size()<<' '<<components.same(0,2)<<'\n';
    auto color=cp::bipartite_coloring(g);
    for(int c:*color)std::cout<<c<<' ';
    std::cout<<'\n';
    g.add_edge(0,2);
    std::cout<<cp::bipartite_coloring(g).has_value()<<'\n'; // 0
}
