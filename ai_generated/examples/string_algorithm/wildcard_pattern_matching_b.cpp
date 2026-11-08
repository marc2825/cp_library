#include <iostream>
#include <vector>
#include "../../string_algorithm/wildcard_pattern_matching.hpp"

int main() {
    auto positions = cp::wildcard_pattern_matching("abacaba", "a?a");
    std::cout << positions[0] << " " << positions[1] << " " << positions[2] << "\n";
}
