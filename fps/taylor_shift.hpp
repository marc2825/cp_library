#pragma once
#include <algorithm>
#include <cstddef>
#include <vector>
#include "detail/common.hpp"

namespace cp {
// f(x+c)の係数列。入力と同じ長さ。n<=Mod、n<=fps_max_size()。
template <int Mod = 998244353, int Root = 3> std::vector<int> taylor_shift(const std::vector<int>& a, long long c) {
    using F = fps_detail::Field<Mod>;
    const auto n = a.size();
    fps_detail::check_precision<Mod, Root>(n);
    if (!n) return {};
    if (n > static_cast<std::size_t>(Mod)) throw std::domain_error("Taylor shift factorial must be invertible");
    std::vector<int> fact(n, 1), invfact(n, 1), left(n), right(n);
    for (std::size_t i = 1; i < n; ++i) fact[i] = F::mul(fact[i - 1], static_cast<int>(i));
    invfact.back() = F::inv(fact.back());
    for (std::size_t i = n - 1; i > 0; --i) invfact[i - 1] = F::mul(invfact[i], static_cast<int>(i));
    int power = 1, shift = F::norm(c);
    for (std::size_t i = 0; i < n; ++i) {
        left[n - 1 - i] = F::mul(F::norm(a[i]), fact[i]);
        right[i] = F::mul(power, invfact[i]); power = F::mul(power, shift);
    }
    auto result = fps_detail::multiply<Mod, Root>(left, right);
    for (std::size_t i = 0; i < n; ++i) left[i] = F::mul(result[n - 1 - i], invfact[i]);
    return left;
}
} // namespace cp
