#pragma once
#include "convolution.hpp"
#include <cstdint>
#include <vector>
namespace cp {
// Exact integer convolution modulo 2^64, using four 16-bit limbs and two NTT primes.
inline std::vector<std::uint64_t> convolution_u64(const std::vector<std::uint64_t> &a,
                                                  const std::vector<std::uint64_t> &b) {
    constexpr std::size_t limit = std::size_t(1) << 24;
    constexpr int p = 469762049, q = 754974721;
    static_assert(UINT64_C(4) * (limit / 2) * 65535 * 65535 < std::uint64_t(p) * q,
                  "CRT product must exceed every limb diagonal at maximum length");
    static_assert(limit <= ntt_max_size<p>() && limit <= ntt_max_size<q, 11>(),
                  "both NTT primes must support the maximum length");
    const auto shape = convolution_detail::shape(a.size(), b.size(), limit);
    if (!shape.first)
        return {};
    std::vector<std::uint64_t> c(shape.first);
    if (std::min(a.size(), b.size()) <= 32) {
        for (std::size_t i = 0; i < a.size(); ++i)
            for (std::size_t j = 0; j < b.size(); ++j)
                c[i + j] += a[i] * b[j];
        return c;
    }
    constexpr auto inv = convolution_detail::power(p, q - 2, q);
    std::vector<std::vector<int>> ap(4, std::vector<int>(shape.second)), bp = ap, aq = ap, bq = ap;
    for (int k = 0; k < 4; ++k) {
        for (std::size_t i = 0; i < a.size(); ++i)
            ap[k][i] = aq[k][i] = (a[i] >> (16 * k)) & 65535;
        for (std::size_t i = 0; i < b.size(); ++i)
            bp[k][i] = bq[k][i] = (b[i] >> (16 * k)) & 65535;
        ntt<p>(ap[k]);
        ntt<p>(bp[k]);
        ntt<q, 11>(aq[k]);
        ntt<q, 11>(bq[k]);
    }
    // Each diagonal has <=4*min(n,m)*65535^2 < p*q under the length bound.
    for (int k = 0; k < 4; ++k) {
        std::vector<int> x(shape.second), y(shape.second);
        for (int j = 0; j <= k; ++j)
            for (std::size_t i = 0; i < shape.second; ++i) {
                x[i] = (x[i] + static_cast<long long>(ap[j][i]) * bp[k - j][i]) % p;
                y[i] = (y[i] + static_cast<long long>(aq[j][i]) * bq[k - j][i]) % q;
            }
        ntt<p>(x, true);
        ntt<q, 11>(y, true);
        for (std::size_t i = 0; i < c.size(); ++i) {
            auto t = (y[i] - static_cast<long long>(x[i]) + q) % q * inv % q;
            auto value = std::uint64_t(x[i]) + std::uint64_t(p) * t;
            c[i] += value << (16 * k);
        }
    }
    return c;
}
} // namespace cp
