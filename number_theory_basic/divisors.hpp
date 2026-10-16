#pragma once
#include <stdexcept>
#include <vector>

namespace cp {
// 正の約数を重複なく昇順で返す。1は{1}。0・負数は拒否。
inline std::vector<long long> divisors(long long n) {
    if (n <= 0) throw std::invalid_argument("divisors require positive n");
    std::vector<long long> small, large;
    for (long long d = 1; d <= n / d; ++d) if (n % d == 0) {
        small.push_back(d);
        if (d != n / d) large.push_back(n / d);
    }
    small.insert(small.end(), large.rbegin(), large.rend());
    return small;
}
} // namespace cp
