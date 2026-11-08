#pragma once
#include "big_rational.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
namespace cp {
// Exact rational value of a finite radix-2 long double, including subnormals.
inline BigRational rational_from_binary(long double x) {
    if (!std::isfinite(x))
        throw std::invalid_argument("finite binary value required");
    static_assert(std::numeric_limits<long double>::radix == 2, "binary floating required");
    if (!x)
        return 0;
    int exponent;
    auto mantissa = std::frexp(std::abs(x), &exponent);
    BigInt::backend_type n = 0, d = 1;
    const int digits = std::numeric_limits<long double>::digits;
    for (int i = 0; i < digits; ++i) {
        mantissa *= 2;
        n <<= 1;
        if (mantissa >= 1) {
            ++n;
            mantissa -= 1;
        }
    }
    exponent -= digits;
    if (exponent >= 0)
        n <<= exponent;
    else
        d <<= -exponent;
    if (x < 0)
        n = -n;
    return {BigInt(std::move(n)), BigInt(std::move(d))};
}
// Approximation from the leading 61 bits; tiny values may underflow to zero.
inline long double rational_to_long_double(const BigRational &x) {
    if (!x)
        return 0;
    auto a = abs(x.numerator()).native(), b = x.denominator().native();
    const int sa = std::max(0, int(boost::multiprecision::msb(a)) - 60),
              sb = std::max(0, int(boost::multiprecision::msb(b)) - 60);
    a >>= sa;
    b >>= sb;
    const auto value =
        std::ldexp(a.convert_to<long double>() / b.convert_to<long double>(), sa - sb) * x.sign();
    if (!std::isfinite(value))
        throw std::overflow_error("rational approximation overflow");
    return value;
}
} // namespace cp
