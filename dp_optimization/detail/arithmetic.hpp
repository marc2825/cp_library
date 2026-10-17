#pragma once
#include <limits>
#include <stdexcept>
#include <type_traits>

namespace cp { namespace detail {
inline long long dp_add(long long a, long long b) {
    if ((b > 0 && a > std::numeric_limits<long long>::max() - b) ||
        (b < 0 && a < std::numeric_limits<long long>::lowest() - b))
        throw std::overflow_error("DP addition overflow");
    return a + b;
}
inline long long dp_subtract(long long a, long long b) {
    if ((b > 0 && a < std::numeric_limits<long long>::lowest() + b) ||
        (b < 0 && a > std::numeric_limits<long long>::max() + b))
        throw std::overflow_error("DP subtraction overflow");
    return a - b;
}
inline long long dp_multiply(long long a, long long b) {
    const auto lo = std::numeric_limits<long long>::lowest(), hi = std::numeric_limits<long long>::max();
    bool overflow = false;
    if (a > 0) overflow = b > 0 ? a > hi / b : b < lo / a;
    else if (a < 0) {
        if (b > 0) overflow = a < lo / b;
        else if (b < 0) overflow = a < hi / b;
    }
    if (overflow) throw std::overflow_error("DP multiplication overflow");
    return a * b;
}
template <class Count> void dp_count_add(Count& a, const Count& b) {
    if constexpr (std::is_integral_v<Count>) {
        static_assert(!std::is_same_v<Count, bool>, "Count must not be bool");
        if (a > std::numeric_limits<Count>::max() - b) throw std::overflow_error("DP count overflow");
    }
    a += b;
}
} } // namespace cp::detail
