#pragma once
#include <cstddef>
#include <limits>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>
#include "detail/arithmetic.hpp"

namespace cp {
struct AssignmentResult {
    long long cost = 0;
    std::vector<int> row_to_column, column_to_row; // -1 for unused columns.
    std::vector<OptimizationWide> row_potential, column_potential;
};
namespace optimization_detail {
template<class Matrix> std::pair<int, int> assignment_shape(const Matrix& a) {
    if (a.size() >= std::size_t(std::numeric_limits<int>::max()))
        throw std::length_error("too many assignment rows");
    const auto columns = a.empty() ? 0 : a.front().size();
    if (columns >= std::size_t(std::numeric_limits<int>::max()))
        throw std::length_error("too many assignment columns");
    for (const auto& row : a)
        if (row.size() != columns) throw std::invalid_argument("ragged assignment matrix");
    return {static_cast<int>(a.size()), static_cast<int>(columns)};
}
// All rows must be assigned to distinct columns. Missing edges have no finite sentinel.
template<class CostAt>
std::optional<AssignmentResult> solve_assignment(int n, int m, CostAt at, bool maximize) {
    if (n > m) return std::nullopt;
    const auto weight = [&](int i, int j) -> std::optional<Wide> {
        const auto cost = at(i, j);
        if (!cost) return std::nullopt;
        const Wide x = *cost;
        return maximize ? sub(0, x) : x;
    };
    std::vector<Wide> u(n + 1), v(m + 1);
    for (int i = 1; i <= n; ++i) {
        std::optional<Wide> minimum;
        for (int j = 0; j < m; ++j) {
            const auto x = weight(i - 1, j);
            if (x && (!minimum || *x < *minimum)) minimum = x;
        }
        if (!minimum) return std::nullopt;
        u[i] = *minimum;
    }
    std::vector<int> matched(m + 1), previous(m + 1);
    for (int row = 1; row <= n; ++row) {
        matched[0] = row;
        int column = 0;
        std::vector<std::optional<Wide>> slack(m + 1);
        std::vector<bool> used(m + 1, false);
        do {
            used[column] = true;
            const int i = matched[column];
            int next = -1;
            std::optional<Wide> delta;
            for (int j = 1; j <= m; ++j) if (!used[j]) {
                const auto x = weight(i - 1, j - 1);
                if (x) {
                    const auto reduced = sub(sub(*x, u[i]), v[j]);
                    if (!slack[j] || reduced < *slack[j]) {
                        slack[j] = reduced;
                        previous[j] = column;
                    }
                }
                // Old slack remains a candidate even if this row has no edge to j.
                if (slack[j] && (!delta || *slack[j] < *delta)) {
                    delta = slack[j]; next = j;
                }
            }
            if (!delta) return std::nullopt;
            for (int j = 0; j <= m; ++j) {
                if (used[j]) { u[matched[j]] = add(u[matched[j]], *delta); v[j] = sub(v[j], *delta); }
                else if (slack[j]) slack[j] = sub(*slack[j], *delta);
            }
            column = next;
        } while (matched[column]);
        do {
            const int parent = previous[column];
            matched[column] = matched[parent];
            column = parent;
        } while (column);
    }
    AssignmentResult result;
    result.row_to_column.assign(n, -1); result.column_to_row.assign(m, -1);
    for (int j = 1; j <= m; ++j) if (matched[j]) {
        result.row_to_column[matched[j] - 1] = j - 1;
        result.column_to_row[j - 1] = matched[j] - 1;
    }
    Wide total = 0;
    for (int i = 0; i < n; ++i) total = add(total, Wide(*at(i, result.row_to_column[i])));
    result.cost = narrow<long long>(total);
    for (int i = 1; i <= n; ++i) result.row_potential.push_back(maximize ? sub(0, u[i]) : u[i]);
    for (int j = 1; j <= m; ++j) result.column_potential.push_back(maximize ? sub(0, v[j]) : v[j]);
    return result;
}
} // namespace optimization_detail
inline std::optional<AssignmentResult> hungarian(const std::vector<std::vector<long long>>& costs) {
    const auto [n, m] = optimization_detail::assignment_shape(costs);
    return optimization_detail::solve_assignment(n, m,
        [&](int i, int j) { return std::optional<long long>(costs[i][j]); }, false);
}
inline std::optional<AssignmentResult> hungarian(const std::vector<std::vector<std::optional<long long>>>& costs) {
    const auto [n, m] = optimization_detail::assignment_shape(costs);
    return optimization_detail::solve_assignment(n, m,
        [&](int i, int j) { return costs[i][j]; }, false);
}
} // namespace cp
