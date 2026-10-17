#include <iostream>
#include "tree/tree_hash.hpp"
int main() {
    cp::Graph<int> g(3);g.add_edge(0,1);g.add_edge(0,2);auto a=cp::rooted_tree_hash(g);
    std::cout<<(a.hash[1]==a.hash[2])<<" "<<cp::unrooted_tree_hash(g).size<<"\n";
}
