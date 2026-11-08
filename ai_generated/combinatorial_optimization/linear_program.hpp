#pragma once
#include "../basic_utility/big_rational.hpp"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <type_traits>
#include <vector>
namespace cp {
enum class LPStatus { optimal, infeasible, unbounded, iteration_limit, numerical_failure };
inline const char *lp_status_name(LPStatus s) {
    switch (s) {
    case LPStatus::optimal:
        return "optimal";
    case LPStatus::infeasible:
        return "infeasible";
    case LPStatus::unbounded:
        return "unbounded";
    case LPStatus::iteration_limit:
        return "iteration_limit";
    case LPStatus::numerical_failure:
        return "numerical_failure";
    }
    return "unknown";
}
struct SimplexOptions {
    std::size_t max_pivots = 100000;
    long double epsilon = 1e-12L;
};
template <class T> struct LinearProgramResult {
    LPStatus status = LPStatus::iteration_limit;
    bool has_primal = false;
    T value = 0;
    std::vector<T> primal, dual, ray; // dual is a Farkas vector when infeasible
    std::size_t pivots = 0;
};
namespace lp_detail {
struct NumericalFailure {};
template <class T> T finite(T x) {
    if constexpr (std::is_floating_point_v<T>)
        if (!std::isfinite(x))
            throw NumericalFailure{};
    return x;
}
template <class T>
void validate(const std::vector<std::vector<T>> &a, const std::vector<T> &b, const std::vector<T> &c) {
    static_assert(!std::is_integral_v<T>, "LP needs an ordered field, not integer division");
    if (a.size() != b.size())
        throw std::invalid_argument("LP row count differs");
    for (const auto &row : a)
        if (row.size() != c.size())
            throw std::invalid_argument("LP column count differs");
    if constexpr (std::is_floating_point_v<T>) {
        for (const auto &row : a)
            for (auto x : row)
                if (!std::isfinite(x))
                    throw std::invalid_argument("nonfinite LP coefficient");
        for (const auto *v : {&b, &c})
            for (auto x : *v)
                if (!std::isfinite(x))
                    throw std::invalid_argument("nonfinite LP coefficient");
    }
}
} // namespace lp_detail
// Maximize c*x subject to A*x<=b and x>=0. Two phases, Bland pivots, no finite big-M.
// BigRational gives exact primal/dual/Farkas/ray certificates; floats use absolute epsilon.
template <class T>
LinearProgramResult<T> linear_program(const std::vector<std::vector<T>> &a, const std::vector<T> &b,
                                      const std::vector<T> &c, SimplexOptions options = {}) {
    lp_detail::validate(a, b, c);
    if (!std::isfinite(options.epsilon) || options.epsilon < 0)
        throw std::invalid_argument("invalid simplex epsilon");
    const auto m = a.size(), n = c.size();
    if (m > (std::numeric_limits<std::size_t>::max() - n) / 2)
        throw std::length_error("LP dimension overflow");
    const auto columns = n + 2 * m;
    std::vector<std::vector<T>> t(m, std::vector<T>(columns));
    std::vector<T> rhs(m);
    std::vector<std::size_t> basis(m);
    auto sign = [&](const T &x) {
        if constexpr (std::is_floating_point_v<T>)
            return x > options.epsilon ? 1 : x < -options.epsilon ? -1 : 0;
        else
            return x > T(0) ? 1 : x < T(0) ? -1 : 0;
    };
    for (std::size_t i = 0; i < m; i++) {
        const T s = b[i] < T(0) ? T(-1) : T(1);
        rhs[i] = s * b[i];
        for (std::size_t j = 0; j < n; j++)
            t[i][j] = s * a[i][j];
        t[i][n + i] = s;
        if (s > T(0))
            basis[i] = n + i;
        else {
            basis[i] = n + m + i;
            t[i][n + m + i] = T(1);
        }
    }
    LinearProgramResult<T> out;
    auto pivot = [&](std::size_t row, std::size_t col) {
        if (out.pivots == options.max_pivots)
            return false;
        const auto divisor = t[row][col];
        for (auto &value : t[row])
            value = lp_detail::finite<T>(value / divisor);
        rhs[row] = lp_detail::finite<T>(rhs[row] / divisor);
        t[row][col] = T(1);
        for (std::size_t i = 0; i < t.size(); i++)
            if (i != row) {
                const auto factor = t[i][col];
                for (std::size_t j = 0; j < columns; j++)
                    t[i][j] = lp_detail::finite<T>(t[i][j] - factor * t[row][j]);
                rhs[i] = lp_detail::finite<T>(rhs[i] - factor * rhs[row]);
                t[i][col] = T(0);
            }
        basis[row] = col;
        ++out.pivots;
        return true;
    };
    auto optimize = [&](const std::vector<T> &cost, std::size_t allowed, std::size_t &entering) {
        for (;;) {
            std::vector<bool> basic(columns);
            for (auto j : basis)
                basic[j] = true;
            entering = columns;
            for (std::size_t j = 0; j < allowed; j++)
                if (!basic[j]) {
                    T reduced = cost[j];
                    for (std::size_t i = 0; i < t.size(); i++)
                        reduced = lp_detail::finite<T>(reduced - cost[basis[i]] * t[i][j]);
                    if (sign(reduced) > 0) {
                        entering = j;
                        break;
                    }
                }
            if (entering == columns)
                return LPStatus::optimal;
            std::size_t leaving = t.size();
            T ratio = 0;
            for (std::size_t i = 0; i < t.size(); i++) {
                if (sign(rhs[i]) < 0)
                    throw lp_detail::NumericalFailure{};
                if (rhs[i] < T(0))
                    rhs[i] = T(0);
                if (sign(t[i][entering]) <= 0)
                    continue;
                const T next = lp_detail::finite<T>(rhs[i] / t[i][entering]);
                const int comparison = sign(next - ratio);
                if (leaving == t.size() || comparison < 0 || (comparison == 0 && basis[i] < basis[leaving])) {
                    leaving = i;
                    ratio = next;
                }
            }
            if (leaving == t.size())
                return LPStatus::unbounded;
            if (!pivot(leaving, entering))
                return LPStatus::iteration_limit;
        }
    };
    auto dual = [&](const std::vector<T> &cost) {
        std::vector<T> y(m);
        for (std::size_t j = 0; j < m; j++)
            for (std::size_t i = 0; i < t.size(); i++)
                y[j] = lp_detail::finite<T>(y[j] + cost[basis[i]] * t[i][n + j]);
        return y;
    };
    auto primal = [&] {
        out.primal.assign(n, T(0));
        for (std::size_t i = 0; i < t.size(); i++)
            if (basis[i] < n)
                out.primal[basis[i]] = rhs[i];
        out.value = T(0);
        for (std::size_t j = 0; j < n; j++)
            out.value = lp_detail::finite<T>(out.value + c[j] * out.primal[j]);
        out.has_primal = true;
    };
    try {
        std::vector<T> cost(columns);
        for (std::size_t j = n + m; j < columns; j++)
            cost[j] = T(-1);
        std::size_t entering = columns;
        out.status = optimize(cost, columns, entering);
        if (out.status != LPStatus::optimal) {
            if (out.status == LPStatus::unbounded)
                out.status = LPStatus::numerical_failure;
            return out;
        }
        T phase_value = 0;
        for (std::size_t i = 0; i < t.size(); i++)
            phase_value = lp_detail::finite<T>(phase_value + cost[basis[i]] * rhs[i]);
        if (sign(phase_value) < 0) {
            out.status = LPStatus::infeasible;
            out.dual = dual(cost);
            return out;
        }
        for (std::size_t i = 0; i < t.size();) {
            if (basis[i] < n + m) {
                ++i;
                continue;
            }
            rhs[i] = T(0);
            std::size_t col = n + m;
            for (std::size_t j = 0; j < n + m; j++)
                if (sign(t[i][j])) {
                    col = j;
                    break;
                }
            if (col == n + m) {
                t.erase(t.begin() + i);
                rhs.erase(rhs.begin() + i);
                basis.erase(basis.begin() + i);
            } else {
                if (!pivot(i, col)) {
                    out.status = LPStatus::iteration_limit;
                    return out;
                }
                ++i;
            }
        }
        std::fill(cost.begin(), cost.end(), T(0));
        std::copy(c.begin(), c.end(), cost.begin());
        out.status = optimize(cost, n + m, entering);
        primal();
        if (out.status == LPStatus::optimal)
            out.dual = dual(cost);
        if (out.status == LPStatus::unbounded) {
            out.ray.assign(n, T(0));
            if (entering < n)
                out.ray[entering] = T(1);
            for (std::size_t i = 0; i < t.size(); i++)
                if (basis[i] < n)
                    out.ray[basis[i]] = -t[i][entering];
        }
    } catch (lp_detail::NumericalFailure) {
        out.status = LPStatus::numerical_failure;
        out.has_primal = false;
        out.primal.clear();
        out.dual.clear();
        out.ray.clear();
    }
    return out;
}
// Free variables x=x_plus-x_minus. Certificates are mapped back to the original coordinates.
template <class T>
LinearProgramResult<T> free_linear_program(const std::vector<std::vector<T>> &a, const std::vector<T> &b,
                                           const std::vector<T> &c, SimplexOptions options = {}) {
    lp_detail::validate(a, b, c);
    const auto n = c.size();
    if (n > std::numeric_limits<std::size_t>::max() / 2)
        throw std::length_error("LP dimension overflow");
    auto rows = a;
    for (auto &row : rows) {
        row.resize(2 * n);
        for (std::size_t j = 0; j < n; j++)
            row[n + j] = -row[j];
    }
    auto cost = c;
    for (auto x : c)
        cost.push_back(-x);
    auto out = linear_program(rows, b, cost, options);
    for (auto *v : {&out.primal, &out.ray})
        if (!v->empty()) {
            for (std::size_t j = 0; j < n; j++)
                (*v)[j] -= (*v)[n + j];
            v->resize(n);
        }
    return out;
}
} // namespace cp
