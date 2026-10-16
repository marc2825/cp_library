#pragma once
#include <cstddef>
#include <stdexcept>
#include <utility>

namespace cp { namespace convolution_detail {
inline bool power_of_two(std::size_t n) { return n && !(n & (n - 1)); }
// 空なら{0,0}。和と次の2冪を作る前に上限を検査する。
inline std::pair<std::size_t, std::size_t> shape(std::size_t a, std::size_t b, std::size_t limit) {
    if (!a || !b) return {0, 0};
    if (a > limit || b > limit || a > limit - b + 1)
        throw std::length_error("convolution result is too long");
    const auto size = a + (b - 1);
    std::size_t padded = 1;
    while (padded < size) {
        if (padded > limit / 2) throw std::length_error("padded transform is too long");
        padded *= 2;
    }
    return {size, padded};
}
constexpr long long power(long long a, long long e, int modulus) {
    long long result = 1;
    while (e) {
        if (e & 1) result = result * a % modulus;
        a = a * a % modulus;
        e >>= 1;
    }
    return result;
}
constexpr bool prime(int p) {
    if (p < 2) return false;
    for (int d = 2; static_cast<long long>(d) * d <= p; ++d) if (p % d == 0) return false;
    return true;
}
constexpr bool primitive_root(int p, int root) {
    if (p < 2 || root < 1 || root >= p) return false;
    int remaining = p - 1;
    for (int d = 2; static_cast<long long>(d) * d <= remaining; ++d) {
        if (remaining % d) continue;
        if (power(root, (p - 1) / d, p) == 1) return false;
        while (remaining % d == 0) remaining /= d;
    }
    return remaining == 1 || power(root, (p - 1) / remaining, p) != 1;
}
template <int Mod, int Root> struct NTTTraits {
    static_assert(Mod >= 2 && Mod <= 2000000000 && prime(Mod), "NTT modulus must be prime in [2,2e9]");
    static_assert(primitive_root(Mod, Root), "NTT root must generate the multiplicative group");
    static constexpr int rank = [] { int r = 0, n = Mod - 1; while (n && !(n & 1)) { ++r; n /= 2; } return r; }();
    static constexpr std::size_t capacity = std::size_t(1) << rank;
};
} } // namespace cp::convolution_detail
