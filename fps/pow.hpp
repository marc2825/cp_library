#pragma once
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <vector>
#include "exp.hpp"

namespace cp {
// 非負整数k（uint64全域）に対するa^k mod x^n。0^0=1。
template <int Mod = 998244353, int Root = 3> std::vector<int> fps_pow(const std::vector<int>& a, std::uint64_t k, std::size_t n) {
    using F = fps_detail::Field<Mod>;
    fps_detail::check_precision<Mod, Root>(n);
    if (!n) return {};
    std::vector<int> result(n);
    if (!k) { result[0] = 1; return result; }
    std::size_t v = 0;
    while (v < std::min(n, a.size()) && !F::norm(a[v])) ++v;
    if (v == std::min(n, a.size()) || (v && k > (n - 1) / v)) return result;
    const auto shift = static_cast<std::size_t>(k * v), length = n - shift;
    std::vector<int> unit(length);
    const int lead = F::norm(a[v]), inverse = F::inv(lead);
    for (std::size_t i = 0; i < length && i < a.size() - v; ++i) unit[i] = F::mul(F::norm(a[v + i]), inverse);
    auto logarithm = fps_log<Mod, Root>(unit, length);
    for (auto& x : logarithm) x = F::mul(x, static_cast<int>(k % Mod));
    auto power = fps_exp<Mod, Root>(logarithm, length);
    const auto scale = F::power(lead, k);
    for (std::size_t i = 0; i < length; ++i) result[shift + i] = F::mul(power[i], scale);
    return result;
}
} // namespace cp
