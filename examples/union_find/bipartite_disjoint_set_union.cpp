#include <iostream>
#include "union_find/bipartite_disjoint_set_union.hpp"
int main() {
    cp::BipartiteDisjointSetUnion d(3); d.add_edge(0,1); d.add_edge(1,2);
    std::cout<<d.add_edge(2,0)<<" "<<d.is_bipartite()<<"\n";
}
