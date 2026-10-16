#define PROBLEM "https://cses.fi/problemset/task/2185"
#include <cstdint>
#include <iostream>
#include <vector>
#include "combinatorics/inclusion_exclusion.hpp"
int main() {
    long long upper; int n; std::cin >> upper >> n; std::vector<long long> primes(n);
    for (auto& p : primes) std::cin >> p;
    const auto count = [&](std::uint64_t mask) {
        long long product = 1;
        for (int i = 0; i < n; ++i) if (mask >> i & 1) {
            if (product > upper / primes[i]) return 0LL;
            product *= primes[i];
        }
        return upper / product;
    };
    std::cout << cp::inclusion_exclusion_union<long long>(n, count) << '\n';
}
