#include <iostream>
#include "union_find/potentialized_disjoint_set_union.hpp"

int main() {
    cp::PotentializedDisjointSetUnion<long long> dsu(4);
    dsu.merge(0, 1, 5);  // p[1] - p[0] = 5
    dsu.merge(1, 2, -2); // p[2] - p[1] = -2
    std::cout << *dsu.diff(0, 2) << ' ' << dsu.size(0) << ' ' << dsu.component_count() << '\n';
    std::cout << (dsu.merge(0, 2, 3) == cp::PotentialMergeResult::AlreadyConsistent) << ' '
              << (dsu.merge(0, 2, 99) == cp::PotentialMergeResult::Contradiction) << '\n';
    if (!dsu.diff(0, 3)) std::cout << "?\n";
}
