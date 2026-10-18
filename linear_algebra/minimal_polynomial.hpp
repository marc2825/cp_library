#pragma once
#include <cstddef>
#include <vector>
#include "black_box.hpp"
#include "matrix.hpp"
#include <utility>

namespace cp {
namespace linear_algebra_detail {
// ベクトルのKrylov依存関係を、係数の履歴も保持して消去する。
template <int Mod, class Apply> std::vector<int> vector_minimal_polynomial(std::size_t n, Apply& apply, std::vector<int> power) {
    using F = linear_recurrence_detail::Arithmetic<Mod>;
    std::vector<std::vector<int>> basis(n), expressions(n);
    for (std::size_t degree = 0; degree <= n; ++degree) {
        auto reduced = power; std::vector<int> expression(degree + 1); expression[degree] = 1;
        bool inserted = false;
        for (std::size_t pivot = 0; pivot < n; ++pivot) {
            if (!reduced[pivot]) continue;
            if (basis[pivot].empty()) {
                const int inverse = F::inverse(reduced[pivot]);
                for (auto& x : reduced) x = F::mul(x, inverse);
                for (auto& x : expression) x = F::mul(x, inverse);
                basis[pivot] = std::move(reduced); expressions[pivot] = std::move(expression); inserted = true; break;
            }
            const int factor = reduced[pivot];
            for (std::size_t j = pivot; j < n; ++j) reduced[j] = F::sub(reduced[j], F::mul(factor, basis[pivot][j]));
            for (std::size_t j = 0; j < expressions[pivot].size(); ++j) expression[j] = F::sub(expression[j], F::mul(factor, expressions[pivot][j]));
        }
        if (!inserted) return expression; // 最高次係数は1のまま。
        power = black_box_apply<Mod>(n, apply, power);
    }
    throw std::logic_error("Krylov dependence missing: callback is not a fixed linear map");
}
} // namespace linear_algebra_detail
// 決定的。各標準基底ベクトルの最小多項式のlcm。最悪O(n^4)。
// 行列全体の最小多項式を返す。Wiedemannの射影多項式と区別する。
template <int Mod = 998244353>
std::vector<int> matrix_minimal_polynomial_mod(const Matrix<int>& a) {
    static_assert(linear_recurrence_detail::prime(Mod), "minimal polynomial requires a prime modulus");
    using F = linear_recurrence_detail::Arithmetic<Mod>;
    if (a.rows() != a.cols()) throw std::invalid_argument("minimal polynomial requires a square matrix");
    if (a.rows() >= std::vector<int>().max_size()) throw std::length_error("minimal polynomial degree too large");
    const auto n = a.rows();
    auto apply = [&](const std::vector<int>& v) {
        std::vector<int> result(n);
        for (std::size_t i = 0; i < n; ++i) for (std::size_t j = 0; j < n; ++j) result[i] = F::add(result[i], F::mul(F::norm(a(i, j)), v[j]));
        return result;
    };
    auto trim = [](std::vector<int>& f) { while (!f.empty() && !f.back()) f.pop_back(); };
    auto divide = [&](std::vector<int> f, const std::vector<int>& g) {
        std::vector<int> quotient(f.size() < g.size() ? 0 : f.size() - g.size() + 1);
        const int inverse = F::inverse(g.back());
        while (f.size() >= g.size()) {
            const auto shift = f.size() - g.size(); const int scale = F::mul(f.back(), inverse); quotient[shift] = scale;
            for (std::size_t i = 0; i < g.size(); ++i) f[shift + i] = F::sub(f[shift + i], F::mul(scale, g[i]));
            trim(f);
        }
        trim(quotient); return std::make_pair(quotient, f);
    };
    std::vector<int> result{1};
    for (std::size_t j = 0; j < n && result.size() <= n; ++j) {
        std::vector<int> unit(n); unit[j] = 1;
        const auto m = linear_algebra_detail::vector_minimal_polynomial<Mod>(n, apply, unit);
        auto x = result, y = m;
        while (!y.empty()) { auto remainder = divide(x, y).second; x = std::move(y); y = std::move(remainder); }
        const auto quotient = divide(result, x).first;
        std::vector<int> next(quotient.size() + m.size() - 1);
        for (std::size_t i = 0; i < quotient.size(); ++i) for (std::size_t k = 0; k < m.size(); ++k)
            next[i + k] = F::add(next[i + k], F::mul(quotient[i], m[k]));
        const int scale = F::inverse(next.back()); for (auto& value : next) value = F::mul(value, scale);
        result = std::move(next);
    }
    return result;
}
} // namespace cp
