#pragma once
#include <algorithm>
#include <cstddef>
#include <optional>
#include <vector>
#include "inv.hpp"

namespace cp {
namespace fps_detail {
template <int Mod> std::optional<int> scalar_sqrt(int a) {
    using F = Field<Mod>;
    if (!a) return 0;
    if (F::power(a, (Mod - 1) / 2) != 1) return std::nullopt;
    int r;
    if (Mod % 4 == 3) r = F::power(a, (Mod + 1) / 4);
    else {
        int s = 0, q = Mod - 1;
        while (!(q & 1)) { ++s; q /= 2; }
        int z = 2;
        while (F::power(z, (Mod - 1) / 2) == 1) ++z;
        int c = F::power(z, q), t = F::power(a, q);
        r = F::power(a, (q + 1) / 2);
        while (t != 1) {
            int i = 1, w = F::mul(t, t);
            while (w != 1) { w = F::mul(w, w); ++i; }
            const int b = F::power(c, std::uint64_t(1) << (s - i - 1));
            r = F::mul(r, b); c = F::mul(b, b); t = F::mul(t, c); s = i;
        }
    }
    return std::min(r, Mod - r);
}
} // namespace fps_detail
// g^2=a mod x^n。存在しなければnullopt。奇素数専用、先頭の根は小さい方。
template <int Mod = 998244353, int Root = 3> std::optional<std::vector<int>> fps_sqrt(const std::vector<int>& a, std::size_t n) {
    static_assert(Mod != 2, "FPS square root requires an odd prime");
    using F = fps_detail::Field<Mod>;
    fps_detail::check_precision<Mod, Root>(n);
    if (!n) return std::vector<int>{};
    std::size_t v = 0;
    while (v < std::min(n, a.size()) && !F::norm(a[v])) ++v;
    if (v == std::min(n, a.size())) return std::vector<int>(n);
    if (v & 1) return std::nullopt;
    const auto root = fps_detail::scalar_sqrt<Mod>(F::norm(a[v]));
    if (!root) return std::nullopt;
    const auto length = n - v;
    std::vector<int> unit(length);
    for (std::size_t i = 0; i < length && i < a.size() - v; ++i) unit[i] = F::norm(a[v + i]);
    std::vector<int> g{*root};
    const int half = (Mod + 1) / 2;
    while (g.size() < length) {
        const auto k = std::min(length, 2 * g.size());
        auto quotient = fps_detail::product_prefix<Mod, Root>(fps_detail::prefix<Mod>(unit, k), fps_inv<Mod, Root>(g, k), k);
        g.resize(k);
        for (std::size_t i = 0; i < k; ++i) g[i] = F::mul(F::add(g[i], quotient[i]), half);
    }
    std::vector<int> result(n);
    std::copy(g.begin(), g.end(), result.begin() + v / 2);
    return result;
}
} // namespace cp
