#include <iostream>
#include "combinatorics/bounded_monotone.hpp"
int main() {
    std::cout << cp::count_bounded_monotone_sequences({0, 1, 1, 2}, {2, 3, 4, 4}) << '\n';
}
