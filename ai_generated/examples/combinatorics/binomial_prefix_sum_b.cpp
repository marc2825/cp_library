#include <iostream>
#include "combinatorics/binomial_prefix_sum.hpp"
int main() {
    cp::BinomialPrefixSum p(5); std::cout << p(5,2) << '\n';
}
