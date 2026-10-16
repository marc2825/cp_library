#include <array>
#include <cstdint>
#include <iostream>
#include "combinatorics/inclusion_exclusion.hpp"
int main() {
    // 全体{0,1,2,3,4}。A={0,1,2}, B={1,2,3}。
    const std::array<unsigned, 2> sets{0b00111, 0b01110};
    const auto intersection = [&](std::uint64_t mask) {
        unsigned bits = 0b11111;
        for (int i = 0; i < 2; ++i) if (mask >> i & 1) bits &= sets[i];
        long long count = 0;
        for (; bits; bits &= bits - 1) ++count;
        return count;
    };
    std::cout << cp::inclusion_exclusion_union<long long>(2, intersection) << '\n';
    std::cout << cp::inclusion_exclusion_complement<long long>(2, intersection) << '\n';
}
