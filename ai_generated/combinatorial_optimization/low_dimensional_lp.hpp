#pragma once
#include "../linear_algebra/gauss_jordan.hpp"
#include "linear_program.hpp"
#include <array>
namespace cp {
struct LowDimensionalLPOptions {
    std::size_t max_bases = 100000;
};
struct LowDimensionalLPResult : LinearProgramResult<BigRational> {
    std::size_t bases = 0;
};
// Exact free-variable LP in D=1..3. Enumerates active bases in a symbolic box [-M,M]^D.
// M is formal infinity; degeneracy and unboundedness do not depend on a numeric box size.
template <std::size_t D>
LowDimensionalLPResult
low_dimensional_lp(const std::vector<std::array<BigRational, D>> &a, const std::vector<BigRational> &b,
                   const std::array<BigRational, D> &c, LowDimensionalLPOptions options = {}) {
    static_assert(D >= 1 && D <= 3, "dimension must be 1, 2 or 3");
    if (a.size() != b.size())
        throw std::invalid_argument("low-dimensional LP row count differs");
    using Q = BigRational;
    struct Affine {
        Q offset = 0, slope = 0;
    };
    auto rows = a;
    std::vector<Affine> rhs;
    for (const auto &v : b)
        rhs.push_back({v, 0});
    for (std::size_t j = 0; j < D; j++)
        for (int s : {-1, 1}) {
            std::array<Q, D> row{};
            row[j] = s;
            rows.push_back(row);
            rhs.push_back({0, 1});
        }
    auto sign = [](const Affine &v) { return v.slope ? v.slope.sign() : v.offset.sign(); };
    LowDimensionalLPResult out;
    Affine best;
    std::vector<Q> best_offset, best_slope;
    std::array<std::size_t, D> indices{};
    for (std::size_t i = 0; i < D; i++)
        indices[i] = i;
    bool exhausted = false;
    while (!exhausted) {
        if (out.bases == options.max_bases) {
            out.status = LPStatus::iteration_limit;
            break;
        }
        ++out.bases;
        Matrix<Q> matrix(D, D);
        std::vector<Q> constant(D), slope(D);
        for (std::size_t i = 0; i < D; i++) {
            constant[i] = rhs[indices[i]].offset;
            slope[i] = rhs[indices[i]].slope;
            for (std::size_t j = 0; j < D; j++)
                matrix(i, j) = rows[indices[i]][j];
        }
        const auto u = gauss_jordan(matrix, constant);
        if (u.status == LinearSystemStatus::unique) {
            const auto v = gauss_jordan(matrix, slope);
            bool feasible = true;
            for (std::size_t i = 0; i < rows.size(); i++) {
                Affine residual{-rhs[i].offset, -rhs[i].slope};
                for (std::size_t j = 0; j < D; j++) {
                    residual.offset += rows[i][j] * u.particular[j];
                    residual.slope += rows[i][j] * v.particular[j];
                }
                if (sign(residual) > 0) {
                    feasible = false;
                    break;
                }
            }
            if (feasible) {
                Affine objective;
                for (std::size_t j = 0; j < D; j++) {
                    objective.offset += c[j] * u.particular[j];
                    objective.slope += c[j] * v.particular[j];
                }
                if (best_offset.empty() ||
                    sign({objective.offset - best.offset, objective.slope - best.slope}) > 0) {
                    best = objective;
                    best_offset = u.particular;
                    best_slope = v.particular;
                }
                if (objective.slope > Q(0)) {
                    out.status = LPStatus::unbounded;
                    break;
                }
            }
        }
        std::size_t i = D;
        while (i && indices[i - 1] == rows.size() - D + i - 1)
            --i;
        if (!i)
            exhausted = true;
        else {
            ++indices[i - 1];
            for (std::size_t j = i; j < D; j++)
                indices[j] = indices[j - 1] + 1;
        }
    }
    if (exhausted)
        out.status = best_offset.empty() ? LPStatus::infeasible : LPStatus::optimal;
    if (!best_offset.empty()) {
        Q scale = 1;
        for (std::size_t i = 0; i < a.size(); i++) {
            Q offset = -b[i], slope = 0;
            for (std::size_t j = 0; j < D; j++) {
                offset += a[i][j] * best_offset[j];
                slope += a[i][j] * best_slope[j];
            }
            if (slope < Q(0))
                scale = std::max(scale, offset / (-slope));
        }
        out.has_primal = true;
        out.primal.resize(D);
        for (std::size_t j = 0; j < D; j++) {
            out.primal[j] = best_offset[j] + scale * best_slope[j];
            out.value += c[j] * out.primal[j];
        }
        if (out.status == LPStatus::unbounded)
            out.ray = best_slope;
    }
    return out;
}
} // namespace cp
