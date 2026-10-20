#include <iostream>
#include "basic_utility/partial_persistent_search.hpp"
int main() {
    cp::PartiallyPersistentArray<int> values(2, 0);
    values.set(0, 2); values.set(1, 5); values.set(0, 7);
    auto time = cp::first_true_version(0, values.time() + 1,
        [&](std::int64_t t) { return values.get(t, 0) >= 4; });
    std::cout << *time << ' ' << values.get(1, 0) << '\n';
    cp::PartiallyPersistentDisjointSetUnion dsu(3);
    dsu.merge(0, 1); dsu.merge(1, 2);
    std::cout << *cp::first_connected_version(dsu, 0, 2) << '\n';
}
