#pragma once
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <vector>
#include "matrix.hpp"
#include "detail/arithmetic.hpp"
#include "detail/prime_field.hpp"

namespace cp {
enum class LinearSystemStatus { no_solution, unique, multiple };
template <class T> struct LinearSystemResult {
    LinearSystemStatus status = LinearSystemStatus::no_solution;
    std::size_t rank = 0;
    std::vector<std::size_t> pivot_columns;
    Matrix<T> reduced; // [A|b]の簡約形。矛盾があっても係数側だけでpivotを取る。
    std::vector<T> particular;
    std::vector<std::vector<T>> nullspace_basis; // 自由列の昇順、各自由変数を1にした基底。
    bool has_solution() const { return status != LinearSystemStatus::no_solution; }
};
namespace linear_algebra_detail {
template <class T> LinearSystemResult<T> solve(const Matrix<T>& a, const std::vector<T>& b, T epsilon) {
    static_assert(!std::is_integral_v<T>, "integer division is not a field: use floating point, a field type, or gauss_jordan_mod");
    if (a.rows() != b.size()) throw std::invalid_argument("linear system right-hand side dimension differs");
    if constexpr (std::is_floating_point_v<T>) if (!std::isfinite(epsilon) || epsilon <= 0) throw std::invalid_argument("epsilon must be positive and finite");
    for (const auto& x : a.data()) input(x);
    for (const auto& x : b) input(x);
    if (a.cols() == static_cast<std::size_t>(-1)) throw std::length_error("augmented matrix column count overflow");
    LinearSystemResult<T> result;
    result.reduced = Matrix<T>(a.rows(), a.cols() + 1);
    auto& r = result.reduced;
    for (std::size_t i = 0; i < a.rows(); ++i) {
        for (std::size_t j = 0; j < a.cols(); ++j) r(i, j) = a(i, j);
        r(i, a.cols()) = b[i];
    }
    const auto zero = [&](const T& x) { if constexpr (std::is_floating_point_v<T>) return std::abs(x) <= epsilon; else return x == T(0); };
    for (std::size_t col = 0; col < a.cols() && result.rank < a.rows(); ++col) {
        auto pivot = result.rank;
        if constexpr (std::is_floating_point_v<T>) {
            for (std::size_t i = result.rank + 1; i < a.rows(); ++i) if (std::abs(r(i, col)) > std::abs(r(pivot, col))) pivot = i;
        } else while (pivot < a.rows() && zero(r(pivot, col))) ++pivot;
        if (pivot == a.rows() || zero(r(pivot, col))) {
            for (std::size_t i = result.rank; i < a.rows(); ++i) r(i, col) = T(0);
            continue;
        }
        const auto row = result.rank;
        if (pivot != row) for (std::size_t j = 0; j <= a.cols(); ++j) std::swap(r(row, j), r(pivot, j));
        const auto divisor = r(row, col);
        if constexpr (std::is_floating_point_v<T>) {
            for (std::size_t j = col + 1; j <= a.cols(); ++j) r(row, j) = finite<T>(r(row, j) / divisor);
        } else {
            const T inverse = T(1) / divisor;
            for (std::size_t j = col + 1; j <= a.cols(); ++j) r(row, j) = multiply(r(row, j), inverse);
        }
        r(row, col) = T(1);
        for (std::size_t i = 0; i < a.rows(); ++i) if (i != row) {
            const auto factor = r(i, col);
            // 小さなfactorも省略しない。大きなpivot行と掛かると寄与が大きくなる。
            for (std::size_t j = col + 1; j <= a.cols(); ++j) subtract(r(i, j), multiply(factor, r(row, j)));
            r(i, col) = T(0);
        }
        result.pivot_columns.push_back(col); ++result.rank;
    }
    for (std::size_t i = result.rank; i < a.rows(); ++i) if (!zero(r(i, a.cols()))) return result;
    result.status = result.rank == a.cols() ? LinearSystemStatus::unique : LinearSystemStatus::multiple;
    result.particular.assign(a.cols(), T(0));
    for (std::size_t i = 0; i < result.rank; ++i) result.particular[result.pivot_columns[i]] = r(i, a.cols());
    for (std::size_t col = 0, p = 0; col < a.cols(); ++col) {
        if (p < result.rank && result.pivot_columns[p] == col) { ++p; continue; }
        std::vector<T> v(a.cols(), T(0)); v[col] = T(1);
        for (std::size_t i = 0; i < result.rank; ++i) subtract(v[result.pivot_columns[i]], r(i, col));
        result.nullspace_basis.push_back(std::move(v));
    }
    return result;
}
} // namespace linear_algebra_detail
// 厳密な体。T(0),T(1),+=,-=,*,/,==が必要。入力は変更しない。
template <class T, std::enable_if_t<!std::is_floating_point_v<T>, int> = 0>
LinearSystemResult<T> gauss_jordan(const Matrix<T>& a, const std::vector<T>& b) { return linear_algebra_detail::solve(a, b, T(0)); }
// 実数は列内の最大絶対値pivot。epsilonは絶対許容誤差（数値的な階数判定）。
template <class T, std::enable_if_t<std::is_floating_point_v<T>, int> = 0>
LinearSystemResult<T> gauss_jordan(const Matrix<T>& a, const std::vector<T>& b, T epsilon = T(1e-10)) { return linear_algebra_detail::solve(a, b, epsilon); }
// 素数Mod=2..2e9。intの係数を正規化し、特解・零空間・簡約形をintで返す。
template <int Mod = 998244353> LinearSystemResult<int> gauss_jordan_mod(const Matrix<int>& a, const std::vector<int>& b) {
    using F = linear_algebra_detail::PrimeField<Mod>;
    Matrix<F> field(a.rows(), a.cols());
    for (std::size_t i = 0; i < a.rows(); ++i) for (std::size_t j = 0; j < a.cols(); ++j) field(i, j) = F(a(i, j));
    const auto solved = gauss_jordan(field, std::vector<F>(b.begin(), b.end()));
    LinearSystemResult<int> result; result.status = solved.status; result.rank = solved.rank; result.pivot_columns = solved.pivot_columns;
    result.reduced = Matrix<int>(solved.reduced.rows(), solved.reduced.cols());
    for (std::size_t i = 0; i < result.reduced.rows(); ++i) for (std::size_t j = 0; j < result.reduced.cols(); ++j) result.reduced(i, j) = solved.reduced(i, j).value;
    for (const auto& x : solved.particular) result.particular.push_back(x.value);
    for (const auto& v : solved.nullspace_basis) { result.nullspace_basis.emplace_back(); for (const auto& x : v) result.nullspace_basis.back().push_back(x.value); }
    return result;
}
} // namespace cp
