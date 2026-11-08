#pragma once
#include <optional>
#include "floor_sum_polynomial.hpp"

namespace cp {
struct MinModLinearResult { BigInt value, index; };
// [left,right), signed arbitrary precision a,b. Earliest attaining index. O(log^2 m).
inline std::optional<MinModLinearResult> min_mod_linear(const BigInt& left, const BigInt& right,
        const BigInt& a, const BigInt& b, const BigInt& modulus) {
    if (left > right || modulus <= 0) throw std::invalid_argument("invalid min-mod interval or modulus");
    if (left == right) return std::nullopt;
    const BigInt n = right - left, offset = a * left + b;
    const BigInt base = floor_sum_big(n, modulus, a, offset);
    BigInt lo = 0, hi = modulus - 1;
    while (lo < hi) {
        const BigInt mid = (lo + hi) / 2;
        const BigInt larger = floor_sum_big(n, modulus, a, offset + modulus - 1 - mid) - base;
        if (larger < n) hi = mid; else lo = mid + 1;
    }
    BigInt r = modulus, s = a - nt_detail::floor_div(a, modulus) * modulus, x = 0, y = 1;
    while (s != 0) {
        const BigInt q = r / s, nr = r - q * s, nx = x - q * y;
        r = s; s = nr; x = y; y = nx;
    }
    const BigInt period = modulus / r;
    BigInt index = ((lo - offset) / r * x) % period;
    if (index < 0) index += period;
    return MinModLinearResult{lo, left + index};
}
} // namespace cp
