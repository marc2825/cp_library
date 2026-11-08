#pragma once
#include <stdexcept>
#include <utility>
#include <vector>

namespace cp {
// 正整数の試し割り。昇順の (素数,指数)、1は空。最悪O(sqrt(n))。
inline std::vector<std::pair<long long, int>> prime_factorization(long long n) {
    if (n <= 0) throw std::invalid_argument("factorization requires positive n");
    std::vector<std::pair<long long, int>> result;
    for (long long p = 2; p <= n / p; p += (p == 2 ? 1 : 2)) {
        if (n % p) continue;
        int exponent = 0;
        do { n /= p; ++exponent; } while (n % p == 0);
        result.emplace_back(p, exponent);
    }
    if (n > 1) result.emplace_back(n, 1);
    return result;
}
} // namespace cp
