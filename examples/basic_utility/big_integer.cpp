#include <iostream>
#include "basic_utility/big_integer.hpp"
int main() {
    cp::BigInt a("123456789012345678901234567890"), b(97);
    auto [q, r] = cp::divmod(a, b);
    std::cout << q << " " << r << "\n";
    std::cout << cp::gcd(a, cp::BigInt(90)) << "\n";
    std::cout << cp::BigInt("-0xFF", 16).to_string(16) << "\n";
    auto [negative_q, negative_r] = cp::divmod(cp::BigInt(-17), cp::BigInt(5));
    std::cout << negative_q << " " << negative_r << "\n";
}
