#include <iostream>
#include <vector>
#include "../../combinatorial_optimization/linear_matroid_parity.hpp"

int main() {
    cp::LinearMatroidPairs pairs{{{1, 0, 0, 0}, {0, 1, 0, 0}}, {{0, 0, 1, 0}, {0, 0, 0, 1}}};
    std::cout << cp::linear_matroid_parity(pairs).size() << "\n";
}
