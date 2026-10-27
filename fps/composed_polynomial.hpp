#pragma once
#include "exp.hpp"
#include "inv.hpp"
#include "polynomial.hpp"
namespace cp {
namespace composed_polynomial_detail {
template <int Mod, int Root> std::vector<int> powers(const std::vector<int> &a, std::size_t n) {
    using F = fps_detail::Field<Mod>;
    auto p = polynomial_trim<Mod>(a);
    if (p.empty())
        throw std::domain_error("composed polynomial needs nonzero inputs");
    auto degree = p.size() - 1;
    int scale = F::inv(p.back());
    std::reverse(p.begin(), p.end());
    for (auto &x : p)
        x = F::mul(x, scale);
    auto r =
        fps_detail::product_prefix<Mod, Root>(polynomial_derivative<Mod>(p), fps_inv<Mod, Root>(p, n), n - 1);
    r.insert(r.begin(), static_cast<int>(degree % Mod));
    for (std::size_t i = 1; i < n; ++i)
        r[i] = F::sub(0, r[i]);
    return r;
}
template <int Mod, int Root>
std::vector<int> compose(const std::vector<int> &a, const std::vector<int> &b, bool sum) {
    using F = fps_detail::Field<Mod>;
    auto x = polynomial_trim<Mod>(a), y = polynomial_trim<Mod>(b);
    if (x.empty() || y.empty())
        throw std::domain_error("composed polynomial needs nonzero inputs");
    const auto m = x.size() - 1, n = y.size() - 1;
    if (!m || !n)
        return {1};
    if (m > (static_cast<std::size_t>(Mod) - 1) / n || m * n >= fps_max_size<Mod, Root>())
        throw std::length_error("composed degree exceeds field/NTT capacity");
    auto d = m * n;
    auto u = powers<Mod, Root>(x, d + 1), v = powers<Mod, Root>(y, d + 1);
    std::vector<int> power(d + 1);
    if (sum) {
        std::vector<int> fact(d + 1, 1), invfact(d + 1, 1);
        for (std::size_t i = 1; i <= d; ++i)
            fact[i] = F::mul(fact[i - 1], static_cast<int>(i));
        invfact[d] = F::inv(fact[d]);
        for (std::size_t i = d; i > 0; --i)
            invfact[i - 1] = F::mul(invfact[i], static_cast<int>(i));
        for (std::size_t i = 0; i <= d; ++i) {
            u[i] = F::mul(u[i], invfact[i]);
            v[i] = F::mul(v[i], invfact[i]);
        }
        power = fps_detail::product_prefix<Mod, Root>(u, v, d + 1);
        for (std::size_t i = 0; i <= d; ++i)
            power[i] = F::mul(power[i], fact[i]);
    } else
        for (std::size_t i = 0; i <= d; ++i)
            power[i] = F::mul(u[i], v[i]);
    auto inv = fps_detail::inverses<Mod>(d);
    power[0] = 0;
    for (std::size_t i = 1; i <= d; ++i)
        power[i] = F::sub(0, F::mul(power[i], inv[i]));
    auto result = fps_exp<Mod, Root>(power, d + 1);
    std::reverse(result.begin(), result.end());
    return result;
}
} // namespace composed_polynomial_detail
// Monic products over all root pairs, with algebraic multiplicities; leading scales discarded.
template <int Mod = 998244353, int Root = 3>
std::vector<int> polynomial_composed_sum(const std::vector<int> &a, const std::vector<int> &b) {
    return composed_polynomial_detail::compose<Mod, Root>(a, b, true);
}
template <int Mod = 998244353, int Root = 3>
std::vector<int> polynomial_composed_product(const std::vector<int> &a, const std::vector<int> &b) {
    return composed_polynomial_detail::compose<Mod, Root>(a, b, false);
}
} // namespace cp
