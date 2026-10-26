#include "number_theory_convolution/prime_sum.hpp"
#include <iostream>
#include "number_theory_convolution/multiplicative_summatory.hpp"

int main() {
    auto prime_count = cp::prime_sum_sieve<cp::BigInt>(100,
        [](auto x) { return cp::BigInt(x); }, [](auto) { return cp::BigInt(1); });
    for (auto x : prime_count.quotients()) prime_count.set(x, 2 * prime_count.sum(x));
    std::cout << cp::multiplicative_summatory(prime_count,
        [](auto, unsigned e, auto) { return cp::BigInt(e + 1); }) << '\n';
}
