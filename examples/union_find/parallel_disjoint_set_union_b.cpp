#include <iostream>
#include "union_find/parallel_disjoint_set_union.hpp"
int main() {
    cp::ParallelDisjointSetUnion d(8); d.merge_ranges(0,4,3);
    std::cout << d.same(1,5) << " " << d.same(3,7) << "\n";
}
