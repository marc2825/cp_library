#pragma once
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <vector>
#include "polynomial.hpp"

namespace cp {
// [x^k]P(x)/Q(x)。Q[0]!=0。deg(P)>=deg(Q)や定数分母も可。
template <int Mod = 998244353, int Root = 3> int bostan_mori(const std::vector<int>& numerator, const std::vector<int>& denominator, std::uint64_t k) {
    using F = fps_detail::Field<Mod>;
    auto p = polynomial_trim<Mod>(numerator), q = polynomial_trim<Mod>(denominator);
    if (q.empty() || !q[0]) throw std::domain_error("Bostan-Mori requires nonzero denominator constant");
    fps_detail::check_precision<Mod, Root>(std::max(p.size(), q.size()));
    while (k && !p.empty()) {
        auto negative = q;
        for (std::size_t i = 1; i < negative.size(); i += 2) negative[i] = F::sub(0, negative[i]);
        const auto s = fps_detail::multiply<Mod, Root>(p, negative), t = fps_detail::multiply<Mod, Root>(q, negative);
        p.clear(); q.clear();
        for (std::size_t i = static_cast<std::size_t>(k & 1); i < s.size(); i += 2) p.push_back(s[i]);
        for (std::size_t i = 0; i < t.size(); i += 2) q.push_back(t[i]);
        fps_detail::trim(p); fps_detail::trim(q); k >>= 1;
    }
    return p.empty() ? 0 : F::mul(p[0], F::inv(q[0]));
}
// a[n]=sum(c[j]*a[n-1-j])。初期値とcは同じ非空長、nは0始まり。
template <int Mod = 998244353, int Root = 3> int linear_recurrence_nth(const std::vector<int>& initial, const std::vector<int>& c, std::uint64_t n) {
    using F = fps_detail::Field<Mod>;
    if (initial.empty() || initial.size() != c.size()) throw std::invalid_argument("recurrence needs equally sized nonempty inputs");
    if (n < initial.size()) return F::norm(initial[static_cast<std::size_t>(n)]);
    if (c.size() >= fps_max_size<Mod, Root>()) throw std::length_error("recurrence degree exceeds FPS capacity");
    std::vector<int> q(c.size() + 1, 1);
    for (std::size_t i = 0; i < c.size(); ++i) q[i + 1] = F::sub(0, F::norm(c[i]));
    auto p = fps_detail::product_prefix<Mod, Root>(initial, q, initial.size());
    return bostan_mori<Mod, Root>(p, q, n);
}
} // namespace cp
