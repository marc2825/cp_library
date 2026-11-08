#pragma once
#include <algorithm>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>
#include "convolution.hpp"

namespace cp {
namespace convolution_detail {
inline std::pair<std::uint64_t, std::uint64_t> absolute_stats(const std::vector<long long>& a) {
    const auto limit = std::uint64_t(1) << 63;
    std::uint64_t sum = 0, maximum = 0;
    for (auto x : a) {
        const auto magnitude = x < 0 ? std::uint64_t(0) - static_cast<std::uint64_t>(x) : static_cast<std::uint64_t>(x);
        maximum = std::max(maximum, magnitude);
        sum = sum > limit || magnitude > limit - sum ? limit + 1 : sum + magnitude;
    }
    return {sum, maximum};
}
} // namespace convolution_detail
// 正確な符号付き64ビット畳み込み。出力長<=2^24。
// min(sum|a|*max|b|,sum|b|*max|a|)<=2^63を要求し、満たさなければoverflow_error。
inline std::vector<long long> convolution_ll(const std::vector<long long>& a, const std::vector<long long>& b) {
    static_assert(std::numeric_limits<long long>::digits == 63, "convolution_ll requires 64-bit long long");
    const auto sizes = convolution_detail::shape(a.size(), b.size(), std::size_t(1) << 24);
    if (!sizes.first) return {};
    const auto sa = convolution_detail::absolute_stats(a), sb = convolution_detail::absolute_stats(b);
    if (!sa.second || !sb.second) return std::vector<long long>(sizes.first);
    const auto limit = std::uint64_t(1) << 63;
    if (sa.first > limit / sb.second && sb.first > limit / sa.second)
        throw std::overflow_error("convolution_ll absolute coefficient bound exceeds 2^63");
    constexpr int p0 = 167772161, p1 = 469762049, p2 = 1224736769;
    constexpr long long p01 = static_cast<long long>(p0) * p1;
    constexpr auto inv0 = convolution_detail::power(p0, p1 - 2, p1);
    constexpr auto inv01 = convolution_detail::power(p01 % p2, p2 - 2, p2);
    auto x = convolution<p0>(a, b), y = convolution<p1>(a, b), z = convolution<p2>(a, b);
    std::vector<long long> result(sizes.first);
    for (std::size_t i = 0; i < result.size(); ++i) {
        const auto t1 = (y[i] - static_cast<long long>(x[i]) + p1) % p1 * inv0 % p1;
        const auto low = x[i] + p0 * t1;
        const auto t2 = (z[i] - low % p2 + p2) % p2 * inv01 % p2;
        // 混合基数の桁でP/2と比較。Pは約2^86なので64ビットでは直接作らない。
        const bool negative = t2 != p2 / 2 ? t2 > p2 / 2 : t1 != p1 / 2 ? t1 > p1 / 2 : x[i] > p0 / 2;
        const auto residue = static_cast<std::uint64_t>(low) + static_cast<std::uint64_t>(p01) * static_cast<std::uint64_t>(t2);
        const auto magnitude = negative ? static_cast<std::uint64_t>(p01) * p2 - residue : residue;
        if (magnitude > limit || (!negative && magnitude == limit)) throw std::overflow_error("convolution coefficient is outside long long");
        result[i] = magnitude == limit ? std::numeric_limits<long long>::min()
                  : negative ? -static_cast<long long>(magnitude) : static_cast<long long>(magnitude);
    }
    return result;
}
} // namespace cp
