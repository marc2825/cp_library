#include <iostream>
#include <vector>
#include "combinatorics/burnside.hpp"

int main() {
    std::cout << cp::burnside_orbits_exact({8,2,2}) << "\n";
}
