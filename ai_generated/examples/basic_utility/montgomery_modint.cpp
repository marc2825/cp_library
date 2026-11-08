#include <iostream>
#include "basic_utility/montgomery_modint.hpp"
int main() {
    using Mint = cp::MontgomeryModInt64<9223372036854775807ULL>;
    std::cout << Mint(-1) * Mint(-1) << " " << Mint(2).pow(60) << "\n";
    cp::MontgomeryReduction64 reducer(101);
    auto product = reducer.multiply_encoded(reducer.encode(12), reducer.encode(13));
    std::cout << reducer.decode(product) << "\n";
}
