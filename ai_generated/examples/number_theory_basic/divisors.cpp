#include <iostream>
#include "number_theory_basic/divisors.hpp"
int main() {
    for (long long d : cp::divisors(36)) std::cout << d << ' ';
    std::cout << '\n';
}
