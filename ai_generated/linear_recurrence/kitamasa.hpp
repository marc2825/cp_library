#pragma once
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <vector>
#include "detail/common.hpp"

namespace cp {
namespace linear_recurrence_detail {
template <int Mod> std::vector<int> combine(const std::vector<int>& a, const std::vector<int>& b, const std::vector<int>& c) {
    using R = Arithmetic<Mod>;
    const auto d = c.size();
    std::vector<int> product(2 * d - 1);
    for (std::size_t i = 0; i < d; ++i) for (std::size_t j = 0; j < d; ++j)
        product[i + j] = R::add(product[i + j], R::mul(a[i], b[j]));
    for (std::size_t i = product.size(); i-- > d;) for (std::size_t j = 0; j < d; ++j)
        product[i - 1 - j] = R::add(product[i - 1 - j], R::mul(product[i], c[j]));
    product.resize(d); return product;
}
} // namespace linear_recurrence_detail
// X^n mod (X^d-c[0]X^(d-1)-...-c[d-1])。初期項の昇順の重みをd個返す。
template <int Mod = 998244353> std::vector<int> kitamasa_coefficients(const std::vector<int>& c, std::uint64_t n) {
    if (c.empty()) throw std::invalid_argument("Kitamasa needs a nonempty recurrence");
    const auto d = c.size();
    if (d > std::vector<int>().max_size() / 2) throw std::length_error("Kitamasa working polynomial is too large");
    const auto normalized = linear_recurrence_detail::normalize<Mod>(c);
    std::vector<int> result(d), base(d); result[0] = 1 % Mod;
    if (d == 1) base[0] = normalized[0]; else base[1] = 1 % Mod;
    for (; n; n >>= 1) {
        if (n & 1) result = linear_recurrence_detail::combine<Mod>(result, base, normalized);
        if (n > 1) base = linear_recurrence_detail::combine<Mod>(base, base, normalized);
    }
    return result;
}
// 既知の漸化式のa[n]。O(d² log(n+1))、合成数法・法1も可。
template <int Mod = 998244353> int kitamasa(const std::vector<int>& initial, const std::vector<int>& c, std::uint64_t n) {
    using R = linear_recurrence_detail::Arithmetic<Mod>;
    linear_recurrence_detail::validate(initial, c);
    if (n < initial.size()) return R::norm(initial[static_cast<std::size_t>(n)]);
    const auto weights = kitamasa_coefficients<Mod>(c, n);
    int result = 0;
    for (std::size_t i = 0; i < initial.size(); ++i) result = R::add(result, R::mul(weights[i], R::norm(initial[i])));
    return result;
}
} // namespace cp
