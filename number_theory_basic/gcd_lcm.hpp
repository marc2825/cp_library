#pragma once
#include <limits>
#include <optional>

namespace cp {
namespace detail {
inline unsigned long long integer_magnitude(long long x) {
    const auto u = static_cast<unsigned long long>(x);
    return x < 0 ? 0ULL - u : u;
}
inline unsigned long long unsigned_gcd(unsigned long long a, unsigned long long b) {
    while (b) { const auto r = a % b; a = b; b = r; }
    return a;
}
} // namespace detail
// LLONG_MIN も受け取る。gcd(0,0)=0。非負の結果は unsigned long long。
inline unsigned long long gcd(long long a, long long b) {
    return detail::unsigned_gcd(detail::integer_magnitude(a), detail::integer_magnitude(b));
}
// unsigned long long に収まらない場合は nullopt。0 を含む場合は 0。
inline std::optional<unsigned long long> lcm(long long a, long long b) {
    auto x = detail::integer_magnitude(a), y = detail::integer_magnitude(b);
    if (!x || !y) return 0ULL;
    x /= detail::unsigned_gcd(x, y);
    if (x > std::numeric_limits<unsigned long long>::max() / y) return std::nullopt;
    return x * y;
}
} // namespace cp
