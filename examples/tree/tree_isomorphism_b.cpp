#include <iostream>
#include "tree/tree_isomorphism.hpp"
int main() {
    cp::Graph<int> a(3),b(3); a.add_edge(0,1); a.add_edge(1,2); b.add_edge(2,0); b.add_edge(0,1);
    std::cout << cp::tree_isomorphism(a,b).isomorphic << '\n';
}
