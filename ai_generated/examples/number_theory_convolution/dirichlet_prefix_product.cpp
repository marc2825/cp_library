#include "basic_utility/big_integer.hpp"
#include <iostream>
#include "number_theory_convolution/dirichlet_prefix_product.hpp"

int main() {
    cp::QuotientPrefix<cp::BigInt> ones(20, [](auto x) { return cp::BigInt(x); });
    std::cout << cp::dirichlet_prefix_product(ones, ones).total() << '\n';
}
