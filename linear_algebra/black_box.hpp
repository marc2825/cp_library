#pragma once
#include <cstddef>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <vector>
#include "../linear_recurrence/kitamasa.hpp"

namespace cp {
namespace linear_algebra_detail {
template <int Mod, class Apply> std::vector<int> black_box_apply(std::size_t dimension, Apply& apply, const std::vector<int>& x) {
    auto result = apply(x);
    if (result.size() != dimension) throw std::invalid_argument("black-box callback returned wrong dimension");
    return linear_recurrence_detail::normalize<Mod>(result);
}
} // namespace linear_algebra_detail
// 同じ線形作用Aを表すapply(x)=A*x。入力を変更せず、出力をn項返すこと。
template <int Mod = 998244353, class Apply> std::vector<int> black_box_krylov_sequence(
    std::size_t dimension, Apply apply, const std::vector<int>& left, const std::vector<int>& right, std::size_t count) {
    using F = linear_recurrence_detail::Arithmetic<Mod>;
    if (left.size() != dimension || right.size() != dimension) throw std::invalid_argument("Krylov vector dimension mismatch");
    const auto u = linear_recurrence_detail::normalize<Mod>(left); auto v = linear_recurrence_detail::normalize<Mod>(right);
    std::vector<int> sequence(count);
    for (std::size_t i = 0; i < count; ++i) {
        for (std::size_t j = 0; j < dimension; ++j) sequence[i] = F::add(sequence[i], F::mul(u[j], v[j]));
        if (i + 1 < count) v = linear_algebra_detail::black_box_apply<Mod>(dimension, apply, v);
    }
    return sequence;
}
// mは昇冪・monic・正次数。m(A)b=0かつm[0]!=0なら逆を多項式で表す。
// 返す候補は必ずA*x=bを検証。nulloptは解なしの証明ではない。
template <int Mod = 998244353, class Apply> std::optional<std::vector<int>> black_box_solve(
    std::size_t dimension, Apply apply, const std::vector<int>& b, const std::vector<int>& annihilator) {
    static_assert(linear_recurrence_detail::prime(Mod), "black-box solve requires a prime modulus");
    using F = linear_recurrence_detail::Arithmetic<Mod>;
    if (b.size() != dimension) throw std::invalid_argument("black-box RHS dimension mismatch");
    const auto m = linear_recurrence_detail::normalize<Mod>(annihilator);
    if (m.size() < 2 || m.back() != 1) throw std::invalid_argument("annihilator must be monic and have positive degree");
    if (!m[0]) return std::nullopt;
    auto power = linear_recurrence_detail::normalize<Mod>(b); const auto rhs = power;
    std::vector<int> result(dimension); const int scale = F::sub(0, F::inverse(m[0]));
    for (std::size_t i = 1; i < m.size(); ++i) {
        const int coefficient = F::mul(scale, m[i]);
        for (std::size_t j = 0; j < dimension; ++j) result[j] = F::add(result[j], F::mul(coefficient, power[j]));
        if (i + 1 < m.size()) power = linear_algebra_detail::black_box_apply<Mod>(dimension, apply, power);
    }
    if (linear_algebra_detail::black_box_apply<Mod>(dimension, apply, result) != rhs) return std::nullopt;
    return result;
}
// 指定多項式がm(A)v=0を満たすことを検証してからA^exponent*vを返す。
template <int Mod = 998244353, class Apply> std::vector<int> black_box_power_apply(
    std::size_t dimension, Apply apply, const std::vector<int>& v, std::uint64_t exponent, const std::vector<int>& annihilator) {
    using F = linear_recurrence_detail::Arithmetic<Mod>;
    if (v.size() != dimension) throw std::invalid_argument("black-box vector dimension mismatch");
    const auto m = linear_recurrence_detail::normalize<Mod>(annihilator);
    if (m.size() < 2 || m.back() != 1) throw std::invalid_argument("annihilator must be monic and have positive degree");
    const auto degree = m.size() - 1; std::vector<int> recurrence(degree);
    for (std::size_t i = 0; i < degree; ++i) recurrence[i] = F::sub(0, m[degree - 1 - i]);
    const auto weights = kitamasa_coefficients<Mod>(recurrence, exponent);
    auto power = linear_recurrence_detail::normalize<Mod>(v); std::vector<int> result(dimension), residual(dimension);
    for (std::size_t i = 0; i <= degree; ++i) {
        for (std::size_t j = 0; j < dimension; ++j) {
            residual[j] = F::add(residual[j], F::mul(m[i], power[j]));
            if (i < degree) result[j] = F::add(result[j], F::mul(weights[i], power[j]));
        }
        if (i < degree) power = linear_algebra_detail::black_box_apply<Mod>(dimension, apply, power);
    }
    for (auto x : residual) if (x) throw std::domain_error("polynomial does not annihilate the supplied vector");
    return result;
}
} // namespace cp
