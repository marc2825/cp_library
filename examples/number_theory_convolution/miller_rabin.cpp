#include <iostream>
#include "number_theory_convolution/miller_rabin.hpp"
int main() {
    std::cout << cp::is_prime_64(18446744073709551557ULL) << ' ' << cp::is_prime_64(341550071728321ULL) << '\n';
}
