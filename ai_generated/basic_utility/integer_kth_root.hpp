#pragma once
// 説明: docs/basic_utility/integer_kth_root.md
#include <cstdint>
#include <stdexcept>

namespace cp {
// floor(n^(1/k)), uint64全域。乗算前に除算で上限を検査。
inline std::uint64_t integer_kth_root(std::uint64_t n, unsigned k) {
    if (!k) throw std::invalid_argument("zero root degree");
    if (n < 2 || k == 1) return n;
    if (k >= 64) return 1;
    auto fits = [=](std::uint64_t x) {
        std::uint64_t p = 1;
        for (unsigned i = 0; i < k; ++i) {
            if (p > n / x) return false;
            p *= x;
        }
        return true;
    };
    std::uint64_t lo = 1, hi = n;
    while (lo < hi) {
        auto mid = lo + (hi - lo) / 2 + (hi - lo) % 2;
        if (fits(mid)) lo = mid; else hi = mid - 1;
    }
    return lo;
}
} // namespace cp
