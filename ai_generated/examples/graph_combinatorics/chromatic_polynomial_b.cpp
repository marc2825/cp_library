#include <iostream>
#include "graph_combinatorics/chromatic_polynomial.hpp"
int main() {
    cp::Graph<long long> g(3); g.add_edge(0,1,1); g.add_edge(1,2,2); g.add_edge(0,2,4);
    auto p=cp::chromatic_polynomial<long long>(g);
    for(std::size_t i=0;i<p.size();++i) { if(i) std::cout << ' '; std::cout << p[i]; }
    std::cout << '\n';
}
