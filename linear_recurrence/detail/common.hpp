#pragma once
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <vector>

namespace cp {
namespace linear_recurrence_detail {
constexpr bool prime(int p) {
    if (p < 2) return false;
    for (int d = 2; d <= p / d; ++d) if (p % d == 0) return false;
    return true;
}
template <int Mod> struct Arithmetic {
    static_assert(Mod >= 1 && Mod <= 2000000000, "recurrence modulus must be in [1, 2000000000]");
    static int norm(long long x) { x %= Mod; return static_cast<int>(x < 0 ? x + Mod : x); }
    static int add(int a, int b) {
        const auto sum = static_cast<long long>(a) + b;
        return static_cast<int>(sum >= Mod ? sum - Mod : sum);
    }
    static int sub(int a, int b) { return a >= b ? a - b : a - b + Mod; }
    static int mul(int a, int b) { return static_cast<int>(static_cast<long long>(a) * b % Mod); }
    static int inverse(int a) {
        static_assert(prime(Mod), "Berlekamp-Massey requires a prime modulus");
        int r = 1;
        for (int e = Mod - 2; e; e >>= 1, a = mul(a, a)) if (e & 1) r = mul(r, a);
        return r;
    }
};
inline void validate(const std::vector<int>& initial, const std::vector<int>& c) {
    if (initial.empty() || initial.size() != c.size())
        throw std::invalid_argument("recurrence needs equally sized nonempty inputs");
}
template <int Mod> std::vector<int> normalize(const std::vector<int>& a) {
    auto b = a; for (auto& x : b) x = Arithmetic<Mod>::norm(x); return b;
}
} // namespace linear_recurrence_detail
} // namespace cp
