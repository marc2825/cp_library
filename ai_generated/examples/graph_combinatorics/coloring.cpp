#include <iostream>
#include "graph_combinatorics/chromatic_number.hpp"
int main() {
    cp::Graph<> g(5);
    for(int v=0;v<5;++v) g.add_edge(v,(v+1)%5);
    const auto result=cp::minimum_coloring(g);
    std::cout<<result.chromatic_number<<'\n';
    for(int c:result.color) std::cout<<c<<' ';
    std::cout<<'\n';
}
