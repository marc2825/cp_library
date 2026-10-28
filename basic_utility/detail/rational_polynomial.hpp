#pragma once
#include "../big_rational.hpp"
#include <utility>
#include <vector>
namespace cp::rational_polynomial_detail {
using Polynomial = std::vector<BigRational>;
inline void trim(Polynomial &p) {
    while (!p.empty() && !p.back())
        p.pop_back();
}
inline Polynomial derivative(const Polynomial &p) {
    Polynomial d;
    for (std::size_t i = 1; i < p.size(); ++i)
        d.push_back(p[i] * BigRational(i));
    trim(d);
    return d;
}
inline std::pair<Polynomial, Polynomial> divide(Polynomial a, const Polynomial &b) {
    if (b.empty())
        throw std::logic_error("zero polynomial divisor");
    Polynomial q(a.size() >= b.size() ? a.size() - b.size() + 1 : 0);
    while (a.size() >= b.size()) {
        const auto k = a.size() - b.size();
        const auto c = a.back() / b.back();
        q[k] = c;
        for (std::size_t j = 0; j < b.size(); ++j)
            a[j + k] -= c * b[j];
        trim(a);
    }
    trim(q);
    return {std::move(q), std::move(a)};
}
inline void normalize(Polynomial &p, bool positive = false) {
    if (p.empty())
        return;
    const auto leading = positive ? abs(p.back()) : p.back();
    for (auto &x : p)
        x /= leading;
}
inline Polynomial gcd(Polynomial a, Polynomial b) {
    while (!b.empty()) {
        auto r = divide(a, b).second;
        a = std::move(b);
        b = std::move(r);
        normalize(b);
    }
    normalize(a);
    return a;
}
inline BigRational evaluate(const Polynomial &p, const BigRational &x) {
    BigRational value;
    for (auto i = p.size(); i > 0; --i)
        value = value * x + p[i - 1];
    return value;
}
} // namespace cp::rational_polynomial_detail
