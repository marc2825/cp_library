#include <iostream>
#include "number_theory_basic/mod_pow.hpp"
int main() {
    std::cout << cp::mod_pow(-2, 10, 1000) << '\n';
    std::cout << cp::mod_multiply(9000000000000000000LL, 9000000000000000000LL,
                                  9223372036854775807LL) << '\n';
}
