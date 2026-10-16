#pragma once
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <vector>
#include "detail/common.hpp"

namespace cp {
namespace linear_recurrence_detail {
template <int Mod> std::vector<std::vector<int>> normalize_matrix(const std::vector<std::vector<int>>& a) {
    for (const auto& row : a) if (row.size() != a.size()) throw std::invalid_argument("transition matrix must be square");
    auto b = a;
    for (auto& row : b) for (auto& x : row) x = Arithmetic<Mod>::norm(x);
    return b;
}
template <int Mod> std::vector<std::vector<int>> multiply_matrix(const std::vector<std::vector<int>>& a, const std::vector<std::vector<int>>& b) {
    using R = Arithmetic<Mod>;
    const auto n = a.size();
    std::vector<std::vector<int>> c(n, std::vector<int>(n));
    for (std::size_t i = 0; i < n; ++i) for (std::size_t k = 0; k < n; ++k) if (a[i][k])
        for (std::size_t j = 0; j < n; ++j) c[i][j] = R::add(c[i][j], R::mul(a[i][k], b[k][j]));
    return c;
}
template <int Mod> std::vector<int> apply_matrix(const std::vector<std::vector<int>>& a, const std::vector<int>& b) {
    using R = Arithmetic<Mod>;
    std::vector<int> c(b.size());
    for (std::size_t i = 0; i < b.size(); ++i) for (std::size_t j = 0; j < b.size(); ++j)
        c[i] = R::add(c[i], R::mul(a[i][j], b[j]));
    return c;
}
} // namespace linear_recurrence_detail
// 正方行列A^k。指数0は単位行列、0×0は空。合成数法・法1も可。
template <int Mod = 998244353> std::vector<std::vector<int>> matrix_power_mod(const std::vector<std::vector<int>>& a, std::uint64_t k) {
    auto base = linear_recurrence_detail::normalize_matrix<Mod>(a);
    std::vector<std::vector<int>> result(a.size(), std::vector<int>(a.size()));
    for (std::size_t i = 0; i < a.size(); ++i) result[i][i] = 1 % Mod;
    for (; k; k >>= 1) {
        if (k & 1) result = linear_recurrence_detail::multiply_matrix<Mod>(result, base);
        if (k > 1) base = linear_recurrence_detail::multiply_matrix<Mod>(base, base);
    }
    return result;
}
// 列ベクトルの状態遷移s(k)=A^k*s(0)。Aの行数とinitialの長さは同じ。
template <int Mod = 998244353> std::vector<int> matrix_power_apply(const std::vector<std::vector<int>>& a, const std::vector<int>& initial, std::uint64_t k) {
    if (a.size() != initial.size()) throw std::invalid_argument("matrix and state dimensions differ");
    auto base = linear_recurrence_detail::normalize_matrix<Mod>(a);
    auto state = linear_recurrence_detail::normalize<Mod>(initial);
    for (; k; k >>= 1) {
        if (k & 1) state = linear_recurrence_detail::apply_matrix<Mod>(base, state);
        if (k > 1) base = linear_recurrence_detail::multiply_matrix<Mod>(base, base);
    }
    return state;
}
// a[t]=Σ c[j]*a[t-1-j]。initialとcは同じ非空長。nは0始まり。
template <int Mod = 998244353> int linear_recurrence_matrix(const std::vector<int>& initial, const std::vector<int>& c, std::uint64_t n) {
    using R = linear_recurrence_detail::Arithmetic<Mod>;
    linear_recurrence_detail::validate(initial, c);
    if (n < initial.size()) return R::norm(initial[static_cast<std::size_t>(n)]);
    const auto d = c.size();
    std::vector<std::vector<int>> transition(d, std::vector<int>(d));
    transition[0] = c;
    for (std::size_t i = 1; i < d; ++i) transition[i][i - 1] = 1 % Mod;
    return matrix_power_apply<Mod>(transition, std::vector<int>(initial.rbegin(), initial.rend()), n - (d - 1))[0];
}
} // namespace cp
