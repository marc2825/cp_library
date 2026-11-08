#pragma once
#include <algorithm>
#include <cstddef>
#include <vector>
#include "detail/common.hpp"

namespace cp {
// a*g=1 mod x^n。n>0ならa[0]!=0。g <- g(2-ag)で精度を倍増。
template <int Mod = 998244353, int Root = 3> std::vector<int> fps_inv(const std::vector<int>& a, std::size_t n) {
    using F = fps_detail::Field<Mod>;
    fps_detail::check_precision<Mod, Root>(n);
    if (!n) return {};
    if (a.empty()) throw std::domain_error("FPS inverse requires a nonzero constant");
    std::vector<int> g{F::inv(F::norm(a[0]))};
    while (g.size() < n) {
        const auto k = std::min(n, 2 * g.size());
        auto e = fps_detail::product_prefix<Mod, Root>(fps_detail::prefix<Mod>(a, k), g, k);
        for (auto& x : e) x = F::sub(0, x);
        e[0] = F::add(e[0], 2 % Mod);
        g = fps_detail::product_prefix<Mod, Root>(g, e, k);
    }
    return g;
}
} // namespace cp
