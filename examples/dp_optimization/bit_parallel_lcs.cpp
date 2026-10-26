#include <iostream>
#include "dp_optimization/bit_parallel_lcs.hpp"
int main() {
    std::cout << cp::bit_parallel_lcs_length("ABCBDAB", "BDCABA") << '\n';
}
