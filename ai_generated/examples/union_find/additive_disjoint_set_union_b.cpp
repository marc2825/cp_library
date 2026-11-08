#include <iostream>
#include "union_find/additive_disjoint_set_union.hpp"
int main() {
    cp::AdditiveDisjointSetUnion<long long> d(3); d.add_component(0,5); d.merge(0,1); d.add_component(1,2);
    std::cout << d.get(0) << " " << d.get(1) << "\n";
}
