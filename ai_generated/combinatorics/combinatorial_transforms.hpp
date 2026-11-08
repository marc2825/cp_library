#pragma once
// 高速 Lah / Euler 変換と逆変換。説明: docs/combinatorics/combinatorial_transforms.md
#include <cstddef>
#include <stdexcept>
#include <vector>
#include "../fps/exp.hpp"
#include "../fps/log.hpp"
namespace cp {
template<int Mod = 998244353, int Root = 3>
std::vector<int> lah_transform(const std::vector<int>& a, bool inverse = false) {
    using F = fps_detail::Field<Mod>;
    const auto n = a.size();
    if (!n) return {};
    fps_detail::check_precision<Mod, Root>(n);
    if (n > static_cast<std::size_t>(Mod)) throw std::domain_error("Lah indices must be below modulus");
    std::vector<int> fac(n, 1), invfac(n, 1), left(n), right(n);
    for (std::size_t i = 1; i < n; ++i) fac[i] = F::mul(fac[i - 1], static_cast<int>(i));
    invfac.back() = F::inv(fac.back());
    for (std::size_t i = n - 1; i; --i) invfac[i - 1] = F::mul(invfac[i], static_cast<int>(i));
    for (std::size_t i = 0; i < n; ++i) {
        if (i) left[i] = F::mul(F::norm(a[i]), F::mul(invfac[i], invfac[i - 1]));
        right[i] = inverse && (i & 1) ? F::sub(0, invfac[i]) : invfac[i];
    }
    auto b = fps_detail::multiply<Mod, Root>(left, right);
    b.resize(n); b[0] = F::norm(a[0]);
    for (std::size_t i = 1; i < n; ++i) b[i] = F::mul(b[i], F::mul(fac[i], fac[i - 1]));
    return b;
}
// a[0]=0、積 prod_{k>=1}(1-x^k)^(-a[k]) の係数。逆は b[0]=1。
template<int Mod = 998244353, int Root = 3>
std::vector<int> euler_transform(const std::vector<int>& a, bool inverse = false) {
    using F = fps_detail::Field<Mod>;
    const auto n = a.size();
    if (!n) return {};
    fps_detail::check_precision<Mod, Root>(n);
    if (F::norm(a[0]) != (inverse ? 1 : 0)) throw std::invalid_argument("invalid Euler constant term");
    auto inv = fps_detail::inverses<Mod>(n - 1);
    std::vector<int> b(n);
    if (!inverse) {
        for (std::size_t d = 1; d < n; ++d) {
            const auto value = F::mul(static_cast<int>(d), F::norm(a[d]));
            for (std::size_t j = d; j < n; j += d) b[j] = F::add(b[j], value);
        }
        for (std::size_t i = 1; i < n; ++i) b[i] = F::mul(b[i], inv[i]);
        return fps_exp<Mod, Root>(b, n);
    }
    b = fps_log<Mod, Root>(a, n);
    for (std::size_t i = 1; i < n; ++i) b[i] = F::mul(b[i], static_cast<int>(i));
    for (std::size_t d = 1; d < n; ++d) {
        const auto value = b[d];
        for (std::size_t j = d + d; j < n; j += d) b[j] = F::sub(b[j], value);
        b[d] = F::mul(value, inv[d]);
    }
    return b;
}
} // namespace cp
