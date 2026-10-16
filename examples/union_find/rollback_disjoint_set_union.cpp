#include <iostream>
#include "union_find/rollback_disjoint_set_union.hpp"
int main() {
    cp::RollbackDisjointSetUnion d(3); auto saved = d.snapshot();
    d.merge(0, 1); d.merge(1, 2); std::cout << d.component_size(0) << ' ';
    d.rollback(saved); std::cout << d.component_count() << '\n';
}
