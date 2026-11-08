#pragma once
#include <algorithm>
#include <cstddef>
#include <stdexcept>
#include <vector>
#include "composition.hpp"
#include "inv.hpp"

namespace cp {
// f(g(x))=x mod x^n。f[0]=0, f[1]!=0。Newton法。
template <int Mod = 998244353, int Root = 3>
std::vector<int> fps_compositional_inverse(const std::vector<int>& f, std::size_t n) {
    using F = fps_detail::Field<Mod>;
    fps_detail::check_precision<Mod, Root>(n);
    if (!n) return {};
    if (f.size() < 2 || F::norm(f[0]) || !F::norm(f[1])) throw std::domain_error("reversion needs f[0]=0 and f[1]!=0");
    const auto a = fps_truncate<Mod>(f, n); const auto derivative = polynomial_derivative<Mod>(a);
    std::vector<int> g(1);
    for (std::size_t length = 1; length < n;) {
        const auto next = std::min(n, 2 * length);
        auto error = fps_compose<Mod, Root>(a, g, next); error[1] = F::sub(error[1], 1);
        const auto slope = fps_compose<Mod, Root>(derivative, g, next);
        const auto delta = fps_detail::product_prefix<Mod, Root>(error, fps_inv<Mod, Root>(slope, next), next);
        g.resize(next);
        for (std::size_t j = 0; j < next; ++j) g[j] = F::sub(g[j], delta[j]);
        length = next;
    }
    return g;
}
} // namespace cp
