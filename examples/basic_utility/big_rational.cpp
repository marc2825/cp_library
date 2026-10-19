#include <iostream>
#include "basic_utility/big_rational.hpp"
int main() {
    auto a = cp::BigRational::parse("6/-8");
    cp::BigRational b(1, 6);
    std::cout << a + b << " " << a * b << "\n";
    std::cout << cp::BigRational::parse("-0/999").to_string() << "\n";
}
