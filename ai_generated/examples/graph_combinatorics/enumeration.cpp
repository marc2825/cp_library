#include <iostream>
#include "graph_combinatorics/triangle_enumeration.hpp"
#include "graph_combinatorics/clique_enumeration.hpp"
int main() {
    cp::Graph<> g(4);
    g.add_edge(0,1); g.add_edge(0,2); g.add_edge(1,2); g.add_edge(2,3);
    cp::enumerate_triangles(g, [](int u,int v,int w) { std::cout<<u<<' '<<v<<' '<<w<<'\n'; return true; });
    int count=0; cp::enumerate_cliques(g,[&](const auto&) { ++count; return true; });
    std::cout<<count<<'\n';
    cp::enumerate_maximal_cliques(g,[](const auto& vertices) {
        for(int v:vertices) std::cout<<v<<' ';
        std::cout<<'\n'; return true;
    });
}
