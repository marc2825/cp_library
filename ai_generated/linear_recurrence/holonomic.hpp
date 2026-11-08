#pragma once
#include "../fps/polynomial_matrix.hpp"
#include "../linear_algebra/gauss_jordan.hpp"
#include "p_recursive.hpp"
#include <limits>
#include <optional>
namespace cp {
// Find a nonzero relation sum_j p[j](t)*a[t+j]=0, order<=order, degree<=degree.
// Finite-sample candidate only; at least as many equations as unknowns-1 required.
template <int Mod = 998244353>
std::optional<std::vector<std::vector<int>>> infer_holonomic(const std::vector<int> &samples,
                                                             std::size_t order, std::size_t degree) {
    using F = fps_detail::Field<Mod>;
    if (!order || samples.size() < order)
        throw std::invalid_argument("holonomic inference needs positive order and sufficient samples");
    if (order == std::numeric_limits<std::size_t>::max() ||
        degree == std::numeric_limits<std::size_t>::max() ||
        (order + 1) > std::vector<int>().max_size() / (degree + 1))
        throw std::length_error("holonomic dimension overflow");
    auto columns = (order + 1) * (degree + 1), rows = samples.size() - order;
    if (rows < columns - 1)
        throw std::invalid_argument("insufficient samples for requested holonomic bounds");
    Matrix<int> matrix(rows, columns);
    for (std::size_t t = 0; t < rows; ++t)
        for (std::size_t j = 0; j <= order; ++j) {
            int power = 1;
            for (std::size_t k = 0; k <= degree; ++k) {
                matrix(t, j * (degree + 1) + k) = F::mul(F::norm(samples[t + j]), power);
                power = F::mul(power, static_cast<int>(t % Mod));
            }
        }
    auto solved = gauss_jordan_mod<Mod>(matrix, std::vector<int>(rows));
    for (const auto &v : solved.nullspace_basis) {
        std::vector<std::vector<int>> p(order + 1, std::vector<int>(degree + 1));
        for (std::size_t j = 0; j <= order; ++j) {
            std::copy_n(v.begin() + j * (degree + 1), degree + 1, p[j].begin());
            p[j] = polynomial_trim<Mod>(p[j]);
        }
        if (!p.back().empty())
            return p;
    }
    return std::nullopt;
}
// Fast large index through polynomial-matrix blocks. Same nonsingularity condition as p_recursive_prefix.
template <int Mod = 998244353, int Root = 3>
int holonomic_nth(const std::vector<int> &initial, const std::vector<std::vector<int>> &p,
                  std::uint64_t index, std::size_t budget = 65536) {
    using F = fps_detail::Field<Mod>;
    if (initial.empty() || p.size() != initial.size() + 1)
        throw std::invalid_argument("holonomic order mismatch");
    if (index < initial.size())
        return F::norm(initial[index]);
    const auto r = initial.size();
    PolynomialMatrix matrix(r, std::vector<std::vector<int>>(r));
    for (std::size_t i = 0; i + 1 < r; ++i)
        matrix[i][i + 1] = p.back();
    for (std::size_t j = 0; j < r; ++j) {
        matrix.back()[j] = p[j];
        for (auto &x : matrix.back()[j])
            x = F::sub(0, F::norm(x));
    }
    auto count = index - r + 1;
    auto numerator = polynomial_matrix_prefix_product<Mod, Root>(matrix, 0, count, budget);
    auto denominator = polynomial_matrix_prefix_product<Mod, Root>({{p.back()}}, 0, count, budget)[0][0];
    if (!denominator)
        throw std::domain_error("holonomic leading coefficient vanishes on generated range");
    int value = 0;
    for (std::size_t j = 0; j < r; ++j)
        value = F::add(value, F::mul(numerator.back()[j], F::norm(initial[j])));
    return F::mul(value, F::inv(denominator));
}
} // namespace cp
