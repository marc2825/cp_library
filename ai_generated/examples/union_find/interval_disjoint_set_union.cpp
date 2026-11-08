#include <iostream>
#include "union_find/interval_disjoint_set_union.hpp"
int main() {
    cp::IntervalDisjointSetUnion d(6); d.merge(1,4); d.merge(3,6);
    std::cout<<d.size(2)<<" "<<d.same(0,5)<<"\n";
}
