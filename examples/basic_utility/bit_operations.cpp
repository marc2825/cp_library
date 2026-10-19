#include <iostream>
#include "basic_utility/bit_operations.hpp"
int main() {
    std::cout << cp::topbit(0) << " " << cp::topbit(40) << " " << cp::lowbit(40) << "\n";
    bool first = true;
    for (auto mask : cp::FixedPopcountMasks(5, 2)) {
        if (!first) std::cout << " ";
        first = false; std::cout << mask;
    }
    std::cout << "\n";
}
