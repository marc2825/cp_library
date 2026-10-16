#pragma once
#include <algorithm>
#include <cstddef>
#include <vector>
#include "log.hpp"

namespace cp {
// exp(a) mod x^n。n>0ならa[0]=0（空も可）。g <- g(1+a-log(g))。
template <int Mod = 998244353, int Root = 3> std::vector<int> fps_exp(const std::vector<int>& a, std::size_t n) {
    using F = fps_detail::Field<Mod>;
    fps_detail::check_precision<Mod, Root>(n);
    if (!n) return {};
    if (!a.empty() && F::norm(a[0])) throw std::domain_error("FPS exponential requires constant zero");
    std::vector<int> g{1};
    while (g.size() < n) {
        const auto k = std::min(n, 2 * g.size());
        auto e = polynomial_subtract<Mod>(fps_detail::prefix<Mod>(a, k), fps_log<Mod, Root>(g, k));
        e[0] = F::add(e[0], 1);
        g = fps_detail::product_prefix<Mod, Root>(g, e, k);
    }
    return g;
}
} // namespace cp
