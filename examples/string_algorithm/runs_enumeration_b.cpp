#include <iostream>
#include <vector>
#include "../../string_algorithm/runs_enumeration.hpp"

int main() {
    auto runs = cp::runs_enumeration("ababab");
    std::cout << runs[0].period << " " << runs[0].left << " " << runs[0].right << "\n";
}
