#pragma once
#include <limits>
#include <stdexcept>

namespace cp {
namespace detail {
inline long long normalized_remainder(long long a, long long modulus) {
    const auto r = a % modulus;
    return r < 0 ? r + modulus : r;
}
// 0<=a,b<modulus。積が入る場合は直接計算、それ以外は加算・倍増。
inline unsigned long long multiply_remainders(unsigned long long a, unsigned long long b,
                                               unsigned long long modulus) {
    if (!b || a <= std::numeric_limits<unsigned long long>::max() / b) return a * b % modulus;
    unsigned long long result = 0;
    const auto add = [modulus](unsigned long long x, unsigned long long y) {
        return x >= modulus - y ? x - (modulus - y) : x + y;
    };
    while (b) {
        if (b & 1) result = add(result, a);
        b >>= 1;
        if (b) a = add(a, a);
    }
    return result;
}
} // namespace detail
// modulus>=1。負数を正規化し、64ビットの積のオーバーフローを避ける。
inline long long mod_multiply(long long a, long long b, long long modulus) {
    if (modulus <= 0) throw std::invalid_argument("modulus must be positive");
    return static_cast<long long>(detail::multiply_remainders(
        static_cast<unsigned long long>(detail::normalized_remainder(a, modulus)),
        static_cast<unsigned long long>(detail::normalized_remainder(b, modulus)),
        static_cast<unsigned long long>(modulus)));
}
// exponent>=0, modulus>=1。0^0=1、modulus=1 なら常に0。
inline long long mod_pow(long long base, long long exponent, long long modulus) {
    if (exponent < 0 || modulus <= 0) throw std::invalid_argument("invalid exponent or modulus");
    auto value = static_cast<unsigned long long>(detail::normalized_remainder(base, modulus));
    auto result = static_cast<unsigned long long>(1 % modulus);
    const auto m = static_cast<unsigned long long>(modulus);
    while (exponent) {
        if (exponent & 1) result = detail::multiply_remainders(result, value, m);
        exponent >>= 1;
        if (exponent) value = detail::multiply_remainders(value, value, m);
    }
    return static_cast<long long>(result);
}
} // namespace cp
