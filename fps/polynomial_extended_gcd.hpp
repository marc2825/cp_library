#pragma once
#include "detail/finite_field_polynomial.hpp"
#include <optional>
namespace cp {
struct PolynomialExtendedGCD {
    std::vector<int> gcd, first, second;
};
// monic gcd = first*a + second*b; (0,0) -> three zero polynomials.
template <int Mod = 998244353>
PolynomialExtendedGCD polynomial_extended_gcd(std::vector<int> a, std::vector<int> b) {
    using A = finite_polynomial_detail::Algebra<Mod>;
    using P = std::vector<int>;
    a = A::trim(std::move(a));
    b = A::trim(std::move(b));
    P u{1}, v{}, s{}, t{1};
    while (!b.empty()) {
        auto qr = A::divmod(a, b);
        auto ns = A::sub(u, A::mul(qr.first, s));
        auto nt = A::sub(v, A::mul(qr.first, t));
        a = std::move(b);
        b = std::move(qr.second);
        u = std::move(s);
        s = std::move(ns);
        v = std::move(t);
        t = std::move(nt);
    }
    if (a.empty())
        return {{}, {}, {}};
    int c = A::F::inv(a.back());
    return {A::scale(a, c), A::scale(u, c), A::scale(v, c)};
}
// Positive-degree modulus required. nullopt iff not a unit.
template <int Mod = 998244353>
std::optional<std::vector<int>> polynomial_mod_inverse(const std::vector<int> &a,
                                                       const std::vector<int> &modulus) {
    using A = finite_polynomial_detail::Algebra<Mod>;
    auto m = A::trim(modulus);
    if (m.size() < 2)
        throw std::invalid_argument("modulus must have positive degree");
    auto r = polynomial_extended_gcd<Mod>(a, m);
    if (r.gcd != std::vector<int>{1})
        return std::nullopt;
    return A::mod(r.first, m);
}
} // namespace cp
