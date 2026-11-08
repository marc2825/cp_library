#include <iostream>
#include "union_find/persistent_disjoint_set_union.hpp"
int main() {
    cp::PersistentDisjointSetUnion d(3); int a=d.merge(0,0,1), b=d.merge(0,1,2);
    std::cout<<d.same(a,0,1)<<" "<<d.same(b,0,1)<<"\n";
}
