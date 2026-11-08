#pragma once
#include "stable_arithmetic.hpp"
#include <vector>
namespace cp::numerical {
struct TridiagonalResult {
    Status status = Status::converged;
    std::vector<long double> solution;
};
// Adjacent-row partial pivoting preserves an upper band of width two. Reusable O(n) factors.
class TridiagonalFactorization {
    std::vector<long double> diagonal_, upper_, second_, multipliers_;
    std::vector<bool> swaps_;
    Status status_ = Status::converged;
    std::size_t singular_index_ = 0;

  public:
    TridiagonalFactorization(std::vector<long double> lower, std::vector<long double> diagonal,
                             std::vector<long double> upper, ErrorTolerance pivot_tolerance = {0, 0})
        : diagonal_(std::move(diagonal)), upper_(std::move(upper)), multipliers_(std::move(lower)) {
        const auto n = diagonal_.size(), off = n ? n - 1 : 0;
        if (upper_.size() != off || multipliers_.size() != off)
            throw std::invalid_argument("tridiagonal dimensions differ");
        pivot_tolerance.validate();
        long double scale = 0;
        for (const auto *v : {&diagonal_, &upper_, &multipliers_})
            for (auto x : *v) {
                detail::input(x);
                scale = std::max(scale, std::abs(x));
            }
        second_.assign(n > 1 ? n - 2 : 0, 0);
        swaps_.assign(off, false);
        singular_index_ = n;
        auto small = [&](long double x) {
            return x == 0 || (scale && std::abs(x) / scale <=
                                           pivot_tolerance.relative + pivot_tolerance.absolute / scale);
        };
        try {
            for (std::size_t i = 0; i < off; ++i) {
                const auto sub = multipliers_[i];
                if (std::abs(diagonal_[i]) >= std::abs(sub)) {
                    if (small(diagonal_[i])) {
                        status_ = Status::singular;
                        singular_index_ = i;
                        return;
                    }
                    const auto factor = sub / diagonal_[i];
                    multipliers_[i] = factor;
                    diagonal_[i + 1] = detail::checked(std::fma(-factor, upper_[i], diagonal_[i + 1]));
                } else {
                    if (small(sub)) {
                        status_ = Status::singular;
                        singular_index_ = i;
                        return;
                    }
                    const auto factor = diagonal_[i] / sub, old_next = diagonal_[i + 1];
                    multipliers_[i] = factor;
                    diagonal_[i] = sub;
                    diagonal_[i + 1] = detail::checked(std::fma(-factor, old_next, upper_[i]));
                    upper_[i] = old_next;
                    swaps_[i] = true;
                    if (i + 1 < off) {
                        second_[i] = upper_[i + 1];
                        upper_[i + 1] = detail::checked(-factor * upper_[i + 1]);
                    }
                }
            }
            if (n && small(diagonal_.back())) {
                status_ = Status::singular;
                singular_index_ = n - 1;
            }
        } catch (detail::Stop s) {
            status_ = s.status;
        }
    }
    Status status() const { return status_; }
    std::size_t size() const { return diagonal_.size(); }
    std::size_t singular_index() const { return singular_index_; }
    std::size_t row_swaps() const { return std::count(swaps_.begin(), swaps_.end(), true); }
    TridiagonalResult solve(std::vector<long double> b) const {
        const auto n = size();
        if (b.size() != n)
            throw std::invalid_argument("tridiagonal RHS dimension differs");
        for (auto x : b)
            detail::input(x);
        if (status_ != Status::converged)
            return {status_, {}};
        try {
            for (std::size_t i = 0; i + 1 < n; ++i) {
                if (swaps_[i])
                    std::swap(b[i], b[i + 1]);
                b[i + 1] = detail::checked(std::fma(-multipliers_[i], b[i], b[i + 1]));
            }
            for (auto i = n; i > 0;) {
                --i;
                if (i + 1 < n)
                    b[i] = detail::checked(std::fma(-upper_[i], b[i + 1], b[i]));
                if (i + 2 < n)
                    b[i] = detail::checked(std::fma(-second_[i], b[i + 2], b[i]));
                b[i] = detail::checked(b[i] / diagonal_[i]);
            }
        } catch (detail::Stop s) {
            return {s.status, {}};
        }
        return {Status::converged, std::move(b)};
    }
};
inline TridiagonalResult solve_tridiagonal(const std::vector<long double> &lower,
                                           const std::vector<long double> &diagonal,
                                           const std::vector<long double> &upper,
                                           const std::vector<long double> &rhs,
                                           ErrorTolerance pivot_tolerance = {0, 0}) {
    return TridiagonalFactorization(lower, diagonal, upper, pivot_tolerance).solve(rhs);
}
} // namespace cp::numerical
