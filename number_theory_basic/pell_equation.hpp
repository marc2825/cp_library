#pragma once
#include <cstddef>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include "../basic_utility/big_integer.hpp"
#include "integer_kth_root.hpp"

namespace cp {
struct PellSolution { BigInt x, y; };
// Minimal positive x,y for x^2-D*y^2=sign. Negative Pell may have no solution.
inline std::optional<PellSolution> pell_fundamental(std::uint64_t D, int sign = 1,
                                                   std::size_t max_iterations = 1000000) {
    const auto root = integer_kth_root(D, 2);
    if (D == 0 || root * root == D || (sign != 1 && sign != -1)) throw std::invalid_argument("Pell requires positive nonsquare D and sign +/-1");
    BigInt m = 0, d = 1, a = root, p0 = 0, p1 = 1, q0 = 1, q1 = 0;
    for (std::size_t iteration = 0; iteration < max_iterations; ++iteration) {
        const BigInt p = a * p1 + p0, q = a * q1 + q0, norm = p * p - BigInt(D) * q * q;
        if (norm == sign) return PellSolution{p, q};
        if (norm == 1 && sign == -1) return std::nullopt;
        p0 = p1; p1 = p; q0 = q1; q1 = q;
        m = d * a - m; d = (BigInt(D) - m * m) / d; a = (BigInt(root) + m) / d;
    }
    throw std::length_error("Pell continued-fraction iteration budget");
}
inline PellSolution pell_solution_power(std::uint64_t D, PellSolution solution, std::uint64_t exponent) {
    const BigInt norm = solution.x * solution.x - BigInt(D) * solution.y * solution.y;
    if (D == 0 || (norm != 1 && norm != -1)) throw std::invalid_argument("invalid Pell solution");
    auto multiply = [D](const PellSolution& a, const PellSolution& b) -> PellSolution {
        return {a.x * b.x + BigInt(D) * a.y * b.y, a.x * b.y + a.y * b.x};
    };
    PellSolution result{1, 0};
    for (; exponent; exponent >>= 1) {
        if (exponent & 1) result = multiply(result, solution);
        if (exponent > 1) solution = multiply(solution, solution);
    }
    return result;
}
} // namespace cp
