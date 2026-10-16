#pragma once
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <type_traits>

namespace cp { namespace combinatorics_detail {
// 0<=a,b<m、m>=1。long long の和を先に作らない。
inline long long add_mod(long long a, long long b, long long m) {
    return a >= m - b ? a - (m - b) : a + b;
}
inline long long subtract_mod(long long a, long long b, long long m) {
    return a >= b ? a - b : m - (b - a);
}
inline bool odd_popcount(std::uint64_t mask) {
    mask ^= mask >> 32; mask ^= mask >> 16; mask ^= mask >> 8;
    mask ^= mask >> 4; mask ^= mask >> 2; mask ^= mask >> 1;
    return (mask & 1) != 0;
}
template <bool Subtract, class T> void accumulate(T& result, const T& value) {
    if constexpr (std::is_integral_v<T>) {
        static_assert(!std::is_same_v<T, bool>, "bool counts are not supported");
        const auto hi = std::numeric_limits<T>::max(), lo = std::numeric_limits<T>::lowest();
        bool overflow;
        if constexpr (std::is_signed_v<T>) {
            if constexpr (Subtract) overflow = (value > 0 && result < lo + value) || (value < 0 && result > hi + value);
            else overflow = (value > 0 && result > hi - value) || (value < 0 && result < lo - value);
        } else {
            if constexpr (Subtract) overflow = result < value;
            else overflow = result > hi - value;
        }
        if (overflow) throw std::overflow_error("inclusion-exclusion accumulation overflow");
    }
    if constexpr (Subtract) result -= value;
    else result += value;
}
} } // namespace cp::combinatorics_detail
