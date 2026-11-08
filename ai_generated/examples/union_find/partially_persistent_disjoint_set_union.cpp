#include <iostream>
#include "union_find/partially_persistent_disjoint_set_union.hpp"
int main() {
    cp::PartiallyPersistentDisjointSetUnion d(3); d.merge(0,1); d.merge(1,2);
    std::cout<<d.same(0,0,1)<<" "<<d.size(1,0)<<" "<<d.size(2,0)<<"\n";
}
