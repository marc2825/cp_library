#include <iostream>
#include "basic_utility/special_integer_enumeration.hpp"
int main() {
    auto print = [](std::uint64_t x) { std::cout << x << ' '; return true; };
    cp::enumerate_palindromic_integers(90, 140, print);
    std::cout << '\n';
    cp::enumerate_digit_restricted_integers(0, 25, {0, 2}, print);
    std::cout << '\n';
    cp::enumerate_generated_products(20, {2, 3}, print);
    std::cout << '\n';
}
