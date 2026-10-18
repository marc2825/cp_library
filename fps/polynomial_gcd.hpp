#pragma once
#include <utility>
#include <vector>
#include "division.hpp"

namespace cp {
// monic gcd。gcd(0,0)=0（空）。Euclid法、最悪O(D M(D))。
template <int Mod = 998244353, int Root = 3>
std::vector<int> polynomial_gcd(const std::vector<int>& a, const std::vector<int>& b) {
    using F = fps_detail::Field<Mod>;
    auto x = polynomial_trim<Mod>(a), y = polynomial_trim<Mod>(b);
    fps_detail::check_precision<Mod, Root>(std::max(x.size(), y.size()));
    while (!y.empty()) { auto r = polynomial_remainder<Mod, Root>(x, y); x = std::move(y); y = std::move(r); }
    if (!x.empty()) { const int scale = F::inv(x.back()); for (auto& value : x) value = F::mul(value, scale); }
    return x;
}
} // namespace cp
