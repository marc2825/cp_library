#pragma once
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <vector>
#include "../../convolution/convolution.hpp"

namespace cp {
namespace fps_detail {
template <int Mod> struct Field {
    static_assert(Mod >= 2 && Mod <= 2000000000 && convolution_detail::prime(Mod),
                  "FPS modulus must be a prime in [2, 2000000000]");
    static int norm(long long x) { x %= Mod; return static_cast<int>(x < 0 ? x + Mod : x); }
    static int add(int a, int b) { return static_cast<int>((static_cast<long long>(a) + b) % Mod); }
    static int sub(int a, int b) { return a >= b ? a - b : a - b + Mod; }
    static int mul(int a, int b) { return static_cast<int>(static_cast<long long>(a) * b % Mod); }
    static int power(int a, std::uint64_t e) {
        int r = 1;
        for (; e; e >>= 1, a = mul(a, a)) if (e & 1) r = mul(r, a);
        return r;
    }
    static int inv(int a) {
        if (!a) throw std::domain_error("zero has no multiplicative inverse");
        return a == 1 ? 1 : power(a, Mod - 2);
    }
};
template <int Mod> std::vector<int> prefix(const std::vector<int>& a, std::size_t n) {
    if (n > std::vector<int>().max_size()) throw std::length_error("FPS length is too large");
    std::vector<int> b(n);
    for (std::size_t i = 0; i < std::min(n, a.size()); ++i) b[i] = Field<Mod>::norm(a[i]);
    return b;
}
inline void trim(std::vector<int>& a) { while (!a.empty() && !a.back()) a.pop_back(); }
template <int Mod, int Root> constexpr std::size_t max_size() {
    constexpr auto cap = ntt_max_size<Mod, Root>();
    return cap == 1 ? 1 : cap / 2;
}
template <int Mod, int Root> void check_precision(std::size_t n) {
    if (n > max_size<Mod, Root>()) throw std::length_error("FPS precision exceeds NTT working capacity");
}
template <int Mod> std::vector<int> inverses(std::size_t n) {
    if (n >= static_cast<std::size_t>(Mod)) throw std::domain_error("integration indices must be less than modulus");
    std::vector<int> v(n + 1);
    if (n) v[1] = 1;
    for (std::size_t i = 2; i <= n; ++i)
        v[i] = Mod - Field<Mod>::mul(static_cast<int>(Mod / i), v[Mod % i]);
    return v;
}
template <int Mod, int Root> std::vector<int> multiply(const std::vector<int>& a, const std::vector<int>& b) {
    const auto shape = convolution_detail::shape(a.size(), b.size(), ntt_max_size<Mod, Root>());
    if (!shape.first) return {};
    if (std::min(a.size(), b.size()) > 32) return convolution<Mod, Root>(a, b);
    std::vector<int> c(shape.first);
    for (std::size_t i = 0; i < a.size(); ++i) for (std::size_t j = 0; j < b.size(); ++j)
        c[i + j] = Field<Mod>::add(c[i + j], Field<Mod>::mul(Field<Mod>::norm(a[i]), Field<Mod>::norm(b[j])));
    return c;
}
template <int Mod, int Root> std::vector<int> product_prefix(const std::vector<int>& a, const std::vector<int>& b, std::size_t n) {
    auto c = multiply<Mod, Root>(prefix<Mod>(a, std::min(a.size(), n)), prefix<Mod>(b, std::min(b.size(), n)));
    c.resize(n);
    return c;
}
} // namespace fps_detail
// Newton法で用いる出力精度の共通上限。既定では2^22。
template <int Mod = 998244353, int Root = 3> constexpr std::size_t fps_max_size() {
    return fps_detail::max_size<Mod, Root>();
}
} // namespace cp
