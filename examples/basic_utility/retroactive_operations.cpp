#include <iostream>
#include "basic_utility/retroactive_operations.hpp"
#include "range_query/monoid_catalog.hpp"
int main() {
    cp::RetroactiveOperations<cp::AffineFunctionMonoid<long long>> history({10, 20, 30});
    history.set(10, {2, 1}); history.set(30, {3, 0});
    std::cout << history.all()(1) << '\n';
    history.set(20, {1, 4}); // Insert into the past: (2*x+1), (x+4), (3*x).
    std::cout << history.through(20)(1) << ' ' << history.all()(1) << '\n';
    history.erase(10);
    std::cout << history.all()(1) << '\n';
}
