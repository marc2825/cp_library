#pragma once
#include "../basic_utility/big_integer.hpp"
#include <optional>
#include <vector>

namespace cp {
// Arbitrary-degree binary polynomial. Bit i is [x^i]; this is not the Conway nimber basis.
class GF2Polynomial {
    using Bits = BigInt::backend_type;
    Bits bits_ = 0;
    explicit GF2Polynomial(Bits bits) : bits_(std::move(bits)) {}

  public:
    GF2Polynomial(std::uint64_t bits = 0) : bits_(bits) {}
    explicit GF2Polynomial(const BigInt &bits) : bits_(bits.native()) {
        if (bits < 0)
            throw std::invalid_argument("GF2 polynomial bit pattern must be nonnegative");
    }
    BigInt bits() const { return BigInt(bits_); }
    std::int64_t degree() const {
        return bits_ == 0 ? -1 : static_cast<std::int64_t>(boost::multiprecision::msb(bits_));
    }
    bool coefficient(std::size_t i) const {
        if (degree() < 0 || i > static_cast<std::uint64_t>(degree()))
            return false;
        return boost::multiprecision::bit_test(bits_, static_cast<unsigned>(i));
    }
    std::vector<int> coefficients() const {
        std::vector<int> out(static_cast<std::size_t>(degree() + 1));
        for (std::size_t i = 0; i < out.size(); ++i)
            out[i] = coefficient(i);
        return out;
    }
    static GF2Polynomial from_coefficients(const std::vector<int> &a) {
        Bits b = 0;
        for (std::size_t i = a.size(); i-- > 0;) {
            b <<= 1;
            if (a[i] % 2)
                b |= 1;
        }
        return GF2Polynomial(std::move(b));
    }
    friend bool operator==(const GF2Polynomial &a, const GF2Polynomial &b) { return a.bits_ == b.bits_; }
    friend bool operator!=(const GF2Polynomial &a, const GF2Polynomial &b) { return !(a == b); }
    friend GF2Polynomial operator+(const GF2Polynomial &a, const GF2Polynomial &b) {
        return GF2Polynomial(Bits(a.bits_ ^ b.bits_));
    }
    friend GF2Polynomial operator-(const GF2Polynomial &a, const GF2Polynomial &b) { return a + b; }
    friend GF2Polynomial operator*(const GF2Polynomial &a, const GF2Polynomial &b) {
        Bits result = 0, x = a.bits_, y = b.bits_;
        while (y != 0) {
            if ((y & 1) != 0)
                result ^= x;
            y >>= 1;
            x <<= 1;
        }
        return GF2Polynomial(std::move(result));
    }
    friend std::pair<GF2Polynomial, GF2Polynomial> divmod(GF2Polynomial a, const GF2Polynomial &b) {
        if (b.bits_ == 0)
            throw std::domain_error("GF2 polynomial division by zero");
        Bits q = 0;
        while (a.degree() >= b.degree()) {
            auto shift = static_cast<std::size_t>(a.degree() - b.degree());
            q ^= Bits(1) << shift;
            a.bits_ ^= b.bits_ << shift;
        }
        return {GF2Polynomial(std::move(q)), std::move(a)};
    }
    friend GF2Polynomial operator/(const GF2Polynomial &a, const GF2Polynomial &b) {
        return divmod(a, b).first;
    }
    friend GF2Polynomial operator%(const GF2Polynomial &a, const GF2Polynomial &b) {
        return divmod(a, b).second;
    }
};
inline GF2Polynomial gf2_polynomial_gcd(GF2Polynomial a, GF2Polynomial b) {
    while (b != GF2Polynomial()) {
        auto r = a % b;
        a = std::move(b);
        b = std::move(r);
    }
    return a;
}
struct GF2PolynomialBezout {
    GF2Polynomial gcd, first, second;
};
inline GF2PolynomialBezout gf2_polynomial_extended_gcd(GF2Polynomial a, GF2Polynomial b) {
    GF2Polynomial x(1), y(0), u(0), v(1);
    while (b != GF2Polynomial()) {
        auto qr = divmod(a, b);
        auto nx = x + qr.first * u, ny = y + qr.first * v;
        a = std::move(b);
        b = std::move(qr.second);
        x = std::move(u);
        y = std::move(v);
        u = std::move(nx);
        v = std::move(ny);
    }
    return {a, x, y};
}
inline std::optional<GF2Polynomial> gf2_polynomial_inverse(const GF2Polynomial &a,
                                                           const GF2Polynomial &modulus) {
    if (modulus.degree() < 1)
        throw std::invalid_argument("GF2 polynomial modulus needs positive degree");
    auto r = gf2_polynomial_extended_gcd(a, modulus);
    return r.gcd == GF2Polynomial(1) ? std::optional<GF2Polynomial>(r.first % modulus) : std::nullopt;
}
inline GF2Polynomial gf2_polynomial_power(GF2Polynomial a, std::uint64_t exponent,
                                          const GF2Polynomial &modulus) {
    if (modulus.degree() < 1)
        throw std::invalid_argument("GF2 polynomial modulus needs positive degree");
    GF2Polynomial result = GF2Polynomial(1) % modulus;
    a = a % modulus;
    for (; exponent; exponent >>= 1) {
        if (exponent & 1)
            result = (result * a) % modulus;
        if (exponent > 1)
            a = (a * a) % modulus;
    }
    return result;
}
inline bool gf2_polynomial_is_irreducible(const GF2Polynomial &f) {
    if (f.degree() < 1)
        return false;
    const auto n = static_cast<std::size_t>(f.degree());
    std::vector<std::size_t> checks;
    auto r = n;
    for (std::size_t p = 2; p <= r / p; ++p)
        if (r % p == 0) {
            checks.push_back(n / p);
            while (r % p == 0)
                r /= p;
        }
    if (r > 1)
        checks.push_back(n / r);
    auto x = GF2Polynomial(2) % f, h = x;
    for (std::size_t i = 1; i <= n; ++i) {
        h = (h * h) % f;
        if (std::find(checks.begin(), checks.end(), i) != checks.end() &&
            gf2_polynomial_gcd(h + x, f) != GF2Polynomial(1))
            return false;
    }
    return h == x;
}
} // namespace cp
