#pragma once
#include <cstddef>
#include <cstdint>
#include <optional>
#include <random>
#include <stdexcept>
#include <vector>
#include "black_box.hpp"
#include "../linear_recurrence/berlekamp_massey.hpp"

namespace cp {
// scalar列left^T A^i rightの最小多項式。行列全体の最小多項式とは限らない。
// 任意の射影に対し2n項のBM、昇冪monic。零列は{1}。
template <int Mod = 998244353, class Apply> std::vector<int> wiedemann_polynomial(
    std::size_t dimension, Apply apply, const std::vector<int>& left, const std::vector<int>& right) {
    using F = linear_recurrence_detail::Arithmetic<Mod>;
    if (dimension > std::vector<int>().max_size() / 2) throw std::length_error("Wiedemann sequence too long");
    const auto c = berlekamp_massey<Mod>(black_box_krylov_sequence<Mod>(dimension, apply, left, right, 2 * dimension));
    std::vector<int> result(c.size() + 1, 1);
    for (std::size_t i = 0; i < c.size(); ++i) result[c.size() - 1 - i] = F::sub(0, c[i]);
    return result;
}
// ランダム射影→BM→候補解→残差確認。固定seedで再現可能。
// 特異行列や射影失敗でnullopt。解が存在しないとの断定には使わない。
template <int Mod = 998244353, class Apply> std::optional<std::vector<int>> wiedemann_solve(
    std::size_t dimension, Apply apply, const std::vector<int>& b, std::size_t attempts = 8,
    std::uint64_t seed = 20261018) {
    using F = linear_recurrence_detail::Arithmetic<Mod>;
    if (b.size() != dimension) throw std::invalid_argument("Wiedemann RHS dimension mismatch");
    bool zero = true; for (auto x : b) if (F::norm(x)) zero = false;
    if (zero) return std::vector<int>(dimension);
    std::mt19937_64 random(seed); std::uniform_int_distribution<int> distribution(0, Mod - 1);
    for (std::size_t trial = 0; trial < attempts; ++trial) {
        std::vector<int> left(dimension); for (auto& x : left) x = distribution(random);
        const auto m = wiedemann_polynomial<Mod>(dimension, apply, left, b);
        if (m.size() < 2) continue;
        const auto result = black_box_solve<Mod>(dimension, apply, b, m);
        if (result) return result;
    }
    return std::nullopt;
}
} // namespace cp
