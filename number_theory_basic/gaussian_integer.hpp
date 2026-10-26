#pragma once
#include <stdexcept>
#include <utility>
#include "detail/advanced_arithmetic.hpp"

namespace cp {
// Z[i], arbitrary precision. Euclidean division rounds each component to nearest integer.
struct GaussianInteger {
    BigInt real = 0, imag = 0;
    GaussianInteger() = default;
    GaussianInteger(BigInt r, BigInt i = 0) : real(std::move(r)), imag(std::move(i)) {}
    BigInt norm() const { return real * real + imag * imag; }
    GaussianInteger conjugate() const { return {real, -imag}; }
    GaussianInteger operator-() const { return {-real, -imag}; }
    friend GaussianInteger operator+(const GaussianInteger& a, const GaussianInteger& b) { return {a.real + b.real, a.imag + b.imag}; }
    friend GaussianInteger operator-(const GaussianInteger& a, const GaussianInteger& b) { return {a.real - b.real, a.imag - b.imag}; }
    friend GaussianInteger operator*(const GaussianInteger& a, const GaussianInteger& b) {
        return {a.real * b.real - a.imag * b.imag, a.real * b.imag + a.imag * b.real};
    }
    friend bool operator==(const GaussianInteger& a, const GaussianInteger& b) { return a.real == b.real && a.imag == b.imag; }
    friend bool operator!=(const GaussianInteger& a, const GaussianInteger& b) { return !(a == b); }
};
inline std::pair<GaussianInteger, GaussianInteger> gaussian_divmod(const GaussianInteger& a, const GaussianInteger& b) {
    const auto norm = b.norm();
    if (norm == 0) throw std::domain_error("Gaussian division by zero");
    const auto product = a * b.conjugate();
    // Half-integers round toward +infinity. This convention fixes all tie cases.
    const GaussianInteger quotient{nt_detail::floor_div(2 * product.real + norm, 2 * norm),
                                   nt_detail::floor_div(2 * product.imag + norm, 2 * norm)};
    return {quotient, a - b * quotient};
}
inline GaussianInteger gaussian_gcd(GaussianInteger a, GaussianInteger b) {
    while (b != GaussianInteger{}) { auto r = gaussian_divmod(a, b).second; a = b; b = r; }
    if (a == GaussianInteger{}) return a;
    while (a.real <= 0 || a.imag < 0) a = {-a.imag, a.real};
    return a;
}
} // namespace cp
