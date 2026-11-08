#pragma once
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>
#include "hungarian.hpp"

namespace cp {
enum class AssignmentObjective { minimize, maximize };
// All cells start forbidden. Each solve recomputes a full-row assignment.
class AssignmentProblem {
    int columns_;
    std::vector<std::vector<std::optional<long long>>> costs_;
    void check(int row, int column) const {
        if (row < 0 || row >= rows() || column < 0 || column >= columns())
            throw std::out_of_range("assignment cell");
    }
public:
    AssignmentProblem(int rows, int columns) : columns_(columns) {
        if (rows < 0 || columns < 0) throw std::invalid_argument("negative assignment dimension");
        if (rows == std::numeric_limits<int>::max() || columns == std::numeric_limits<int>::max())
            throw std::length_error("assignment dimension is too large");
        costs_.assign(rows, std::vector<std::optional<long long>>(columns));
    }
    explicit AssignmentProblem(std::vector<std::vector<std::optional<long long>>> costs)
        : columns_(optimization_detail::assignment_shape(costs).second), costs_(std::move(costs)) {}
    int rows() const { return static_cast<int>(costs_.size()); }
    int columns() const { return columns_; }
    void set_cost(int row, int column, long long cost) { check(row, column); costs_[row][column] = cost; }
    void forbid(int row, int column) { check(row, column); costs_[row][column] = std::nullopt; }
    std::optional<long long> cost(int row, int column) const { check(row, column); return costs_[row][column]; }
    std::optional<AssignmentResult> solve(AssignmentObjective objective = AssignmentObjective::minimize) const {
        if (objective != AssignmentObjective::minimize && objective != AssignmentObjective::maximize)
            throw std::invalid_argument("unknown assignment objective");
        return optimization_detail::solve_assignment(rows(), columns(),
            [&](int i, int j) { return costs_[i][j]; }, objective == AssignmentObjective::maximize);
    }
};
} // namespace cp
