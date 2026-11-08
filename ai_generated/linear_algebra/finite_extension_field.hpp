#pragma once
#include "../fps/polynomial_extended_gcd.hpp"

namespace cp {
// Deterministic Rabin test over the prime field. Nonzero scalar multiples are accepted.
template <int Mod> bool finite_field_polynomial_is_irreducible(const std::vector<int> &input) {
    using A = finite_polynomial_detail::Algebra<Mod>;
    auto f = A::monic(input);
    if (f.size() < 2)
        return false;
    const auto n = f.size() - 1;
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
    auto x = A::mod({0, 1}, f), h = x;
    for (std::size_t i = 1; i <= n; ++i) {
        h = A::power(h, Mod, f);
        if (std::find(checks.begin(), checks.end(), i) != checks.end() && A::gcd(A::sub(h, x), f).size() != 1)
            return false;
    }
    return h == x;
}
// F_p[x]/f(x) in a user-selected polynomial basis. No global modulus or implicit field conversion.
template <int Mod> class FiniteExtensionField {
    using A = finite_polynomial_detail::Algebra<Mod>;
    std::vector<int> modulus_;

  public:
    using Element = std::vector<int>;
    explicit FiniteExtensionField(Element modulus) : modulus_(A::monic(std::move(modulus))) {
        if (!finite_field_polynomial_is_irreducible<Mod>(modulus_))
            throw std::invalid_argument("extension field modulus must be irreducible and nonconstant");
    }
    const Element &modulus() const { return modulus_; }
    std::size_t degree() const { return modulus_.size() - 1; }
    Element element(const Element &a) const {
        auto r = A::mod(a, modulus_);
        r.resize(degree());
        return r;
    }
    Element zero() const { return Element(degree()); }
    Element one() const { return element({1}); }
    Element add(const Element &a, const Element &b) const { return element(A::add(element(a), element(b))); }
    Element subtract(const Element &a, const Element &b) const {
        return element(A::sub(element(a), element(b)));
    }
    Element multiply(const Element &a, const Element &b) const {
        return element(A::mul(element(a), element(b)));
    }
    Element inverse(const Element &a) const {
        auto r = polynomial_mod_inverse<Mod>(element(a), modulus_);
        if (!r)
            throw std::domain_error("extension field division by zero");
        return element(*r);
    }
    Element divide(const Element &a, const Element &b) const { return multiply(a, inverse(b)); }
    Element power(const Element &a, std::uint64_t exponent) const {
        return element(A::power(element(a), exponent, modulus_));
    }
};
} // namespace cp
