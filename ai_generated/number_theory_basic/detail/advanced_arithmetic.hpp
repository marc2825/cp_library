#pragma once
#include <cstdint>
#include <stdexcept>
#include "../../basic_utility/big_integer.hpp"
#include "../../number_theory_convolution/miller_rabin.hpp"

namespace cp { namespace nt_detail {
using U64 = std::uint64_t;
inline BigInt floor_div(BigInt a, const BigInt& m) {
    BigInt q = a / m;
    if (a % m < 0) --q;
    return q;
}
inline U64 inverse(U64 a, U64 m) {
    if (!m) throw std::invalid_argument("zero modulus");
    BigInt r = m, s = a % m, x = 0, y = 1;
    while (s != 0) {
        const BigInt q = r / s, nr = r - q * s, nx = x - q * y;
        r = s; s = nr; x = y; y = nx;
    }
    if (r != 1) throw std::domain_error("nonunit has no inverse");
    x %= BigInt(m); if (x < 0) x += BigInt(m);
    return x.to_integral<U64>();
}
inline U64 mul(U64 a, U64 b, U64 m) { return factorization_detail::multiply(a, b, m); }
inline U64 pow(U64 a, U64 b, U64 m) { return factorization_detail::power(a, b, m); }
inline BigInt power(BigInt a, unsigned e) {
    BigInt r = 1;
    for (; e; e >>= 1, a *= a) if (e & 1) r *= a;
    return r;
}
} } // namespace cp::nt_detail
