#include <iostream>
#include "number_theory_basic/segmented_sieve.hpp"

int main() {
    cp::SegmentedSieve sieve(100, 111);
    for (auto p : sieve.primes()) std::cout << p << ' ';
    std::cout << '\n';
    for (auto [p, e] : sieve.factorization(108)) std::cout << p << ' ' << e << ' ';
    std::cout << '\n';
}
