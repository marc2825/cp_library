#include "basic_utility/big_integer.hpp"
#include <iostream>
#include "number_theory_convolution/dirichlet_inverse.hpp"

int main() {
    const auto unit = [](cp::BigInt x) -> std::optional<cp::BigInt> {
        return x == 1 ? std::optional<cp::BigInt>(1) : std::nullopt;
    };
    const auto inverse = cp::dirichlet_inverse(std::vector<cp::BigInt>(11, 1), unit);
    for (int i = 1; i <= 10; ++i) std::cout << (*inverse)[i] << ' ';
    std::cout << '\n';
    cp::QuotientPrefix<cp::BigInt> ones(20, [](auto x) { return cp::BigInt(x); });
    std::cout << cp::dirichlet_prefix_inverse(ones, unit)->total() << '\n';
}
