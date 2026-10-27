#pragma once
#include "detail/common.hpp"
#include <algorithm>
#include <array>
#include <cstdint>
#include <stdexcept>
#include <vector>
namespace cp {
// Goldilocks prime 2^64-2^32+1. GCC/Clang unsigned 128-bit multiplication, division-free reduction.
namespace ntt64_detail {
constexpr std::uint64_t modulus = UINT64_C(18446744069414584321), epsilon = UINT64_C(4294967295);
inline std::uint64_t add(std::uint64_t a, std::uint64_t b) {
    auto s = a + b;
    if (s < a)
        s += epsilon;
    return s >= modulus ? s - modulus : s;
}
inline std::uint64_t sub(std::uint64_t a, std::uint64_t b) { return a >= b ? a - b : a - b - epsilon; }
inline std::uint64_t mul(std::uint64_t a, std::uint64_t b) {
    __extension__ using U = unsigned __int128;
    U p = U(a) * b;
    auto lo = std::uint64_t(p), hi = std::uint64_t(p >> 64);
    auto h = hi >> 32, t = lo - h;
    if (lo < h)
        t -= epsilon;
    auto v = (hi & epsilon) * epsilon;
    return add(t, v);
}
inline std::uint64_t power(std::uint64_t a, std::uint64_t e) {
    std::uint64_t r = 1;
    for (; e; e >>= 1, a = mul(a, a))
        if (e & 1)
            r = mul(r, a);
    return r;
}
} // namespace ntt64_detail
inline constexpr std::uint64_t ntt64_modulus = ntt64_detail::modulus;
inline void ntt64(std::vector<std::uint64_t> &a, bool inverse = false) {
    using namespace ntt64_detail;
    const auto n = a.size();
    if (!n)
        return;
    if (!convolution_detail::power_of_two(n))
        throw std::invalid_argument("NTT64 length must be power of two");
    if (std::uint64_t(n) > (UINT64_C(1) << 32))
        throw std::length_error("NTT64 capacity is 2^32");
    for (auto &x : a)
        if (x >= modulus)
            x -= modulus;
    static const auto roots = [] {
        std::array<std::array<std::uint64_t, 2>, 33> r{};
        r[32] = {power(7, epsilon), power(power(7, epsilon), modulus - 2)};
        for (int k = 32; k > 0; --k)
            for (int d = 0; d < 2; ++d)
                r[k - 1][d] = mul(r[k][d], r[k][d]);
        return r;
    }();
    for (std::size_t i = 1, j = 0; i < n; ++i) {
        auto bit = n >> 1;
        for (; j & bit; bit >>= 1)
            j ^= bit;
        j ^= bit;
        if (i < j)
            std::swap(a[i], a[j]);
    }
    int stage = 1;
    for (std::size_t len = 2; len <= n; len *= 2, ++stage) {
        auto root = roots[stage][inverse];
        for (std::size_t i = 0; i < n; i += len) {
            std::uint64_t w = 1;
            for (std::size_t j = 0; j < len / 2; ++j) {
                auto u = a[i + j], v = mul(a[i + j + len / 2], w);
                a[i + j] = add(u, v);
                a[i + j + len / 2] = sub(u, v);
                w = mul(w, root);
            }
        }
        if (len == n)
            break;
    }
    if (inverse) {
        auto c = power(n, modulus - 2);
        for (auto &x : a)
            x = mul(x, c);
    }
}
inline std::vector<std::uint64_t> convolution_ntt64(std::vector<std::uint64_t> a,
                                                    std::vector<std::uint64_t> b) {
    const auto shape =
        convolution_detail::shape(a.size(), b.size(),
                                  static_cast<std::size_t>(std::min<std::uint64_t>(
                                      UINT64_C(1) << 32, std::vector<std::uint64_t>().max_size())));
    if (!shape.first)
        return {};
    a.resize(shape.second);
    b.resize(shape.second);
    ntt64(a);
    ntt64(b);
    for (std::size_t i = 0; i < a.size(); ++i)
        a[i] = ntt64_detail::mul(a[i], b[i]);
    ntt64(a, true);
    a.resize(shape.first);
    return a;
}
} // namespace cp
