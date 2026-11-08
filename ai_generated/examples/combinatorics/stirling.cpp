#include <iostream>
#include <vector>
#include "combinatorics/stirling.hpp"

int main() {
    auto first=cp::stirling_first(5); auto second=cp::stirling_second(5);
    std::cout << first[5][2] << " " << second[5][2] << "\n";
}
