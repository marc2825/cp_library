#pragma once
#include "gcd_lcm.hpp"

namespace cp {
struct ExtendedGCDResult {
    unsigned long long gcd;
    long long x, y;
};
// a*x+b*y=gcd>=0。LLONG_MIN を含む全 long long に対応。詳細は同名の docs。
inline ExtendedGCDResult extended_gcd(long long a, long long b) {
    auto r0 = detail::integer_magnitude(a), r1 = detail::integer_magnitude(b);
    long long x0 = 1, y0 = 0, x1 = 0, y1 = 1;
    if (!r0 && !r1) return {0, 0, 0};
    while (r1) {
        const auto remainder = r0 % r1;
        // 余り0の係数は不要。その計算を避け、|LLONG_MIN| の係数を作らない。
        if (!remainder) { r0 = r1; x0 = x1; y0 = y1; break; }
        const auto quotient = static_cast<long long>(r0 / r1);
        const auto x = x0 - quotient * x1, y = y0 - quotient * y1;
        r0 = r1; r1 = remainder;
        x0 = x1; x1 = x; y0 = y1; y1 = y;
    }
    return {r0, a < 0 ? -x0 : x0, b < 0 ? -y0 : y0};
}
} // namespace cp
