#pragma once
#include "detail/integer_matrix.hpp"
#include "detail/smith_reduction.hpp"
#include "matrix_product.hpp"

namespace cp {
struct SmithNormalForm {
    Matrix<BigInt> diagonal, left, right; // left * input * right == diagonal
    std::size_t rank;
};
namespace integer_matrix_detail {
struct Ring {
    using value_type = BigInt;
    static BigInt zero() { return 0; }
    static BigInt one() { return 1; }
    static BigInt add(const BigInt &a, const BigInt &b) { return a + b; }
    static BigInt sub(const BigInt &a, const BigInt &b) { return a - b; }
    static BigInt mul(const BigInt &a, const BigInt &b) { return a * b; }
    static auto divmod(const BigInt &a, const BigInt &b) { return cp::divmod(a, b); }
    static BigInt normalizing_unit(const BigInt &a) { return a < 0 ? -1 : 1; }
    static BigInt inverse_unit(const BigInt &a) { return a; }
};
} // namespace integer_matrix_detail
// Positive invariant factors d[i] | d[i+1]; unimodular transformations. Zero rows/columns retained.
template <class T> SmithNormalForm smith_normal_form(const Matrix<T> &a) {
    auto r = normal_form_detail::smith<integer_matrix_detail::Ring>(integer_matrix_detail::convert(a));
    return {std::move(r.diagonal), std::move(r.left), std::move(r.right), r.rank};
}
struct IntegerLinearSolution {
    bool solvable = false;
    std::vector<BigInt> particular;
    std::vector<std::vector<BigInt>> kernel_basis; // arbitrary integer coefficients
};
template <class T, class B>
IntegerLinearSolution solve_integer_system(const Matrix<T> &a, const std::vector<B> &b) {
    if (b.size() != a.rows())
        throw std::invalid_argument("integer system RHS dimension mismatch");
    const auto s = smith_normal_form(a);
    std::vector<BigInt> rhs;
    for (const auto &x : b)
        rhs.emplace_back(x);
    auto c = matrix_vector_multiply(s.left, rhs);
    std::vector<BigInt> y(a.cols());
    for (std::size_t i = 0; i < s.rank; ++i) {
        auto qr = divmod(c[i], s.diagonal(i, i));
        if (qr.second != 0)
            return {};
        y[i] = std::move(qr.first);
    }
    for (std::size_t i = s.rank; i < a.rows(); ++i)
        if (c[i] != 0)
            return {};
    IntegerLinearSolution out{true, matrix_vector_multiply(s.right, y), {}};
    for (std::size_t j = s.rank; j < a.cols(); ++j) {
        std::vector<BigInt> v(a.cols());
        for (std::size_t i = 0; i < a.cols(); ++i)
            v[i] = s.right(i, j);
        out.kernel_basis.push_back(std::move(v));
    }
    return out;
}
struct CongruenceLinearSolution {
    bool solvable = false;
    std::vector<BigInt> particular;
    std::vector<std::vector<BigInt>> generators;
    std::vector<BigInt> orders; // x=particular+sum t[i]*generators[i], 0<=t[i]<orders[i], modulo m
};
template <class T, class B>
CongruenceLinearSolution solve_congruence_system(const Matrix<T> &a, const std::vector<B> &b,
                                                 const BigInt &modulus) {
    if (modulus <= 0)
        throw std::invalid_argument("congruence modulus must be positive");
    if (b.size() != a.rows())
        throw std::invalid_argument("congruence RHS dimension mismatch");
    using integer_matrix_detail::residue;
    const auto s = smith_normal_form(a);
    std::vector<BigInt> rhs;
    for (const auto &x : b)
        rhs.emplace_back(x);
    auto c = matrix_vector_multiply(s.left, rhs);
    std::vector<BigInt> y(a.cols()), orders(a.cols(), modulus), steps(a.cols(), 1);
    for (std::size_t i = 0; i < s.rank; ++i) {
        auto g = gcd(s.diagonal(i, i), modulus);
        if (c[i] % g != 0)
            return {};
        auto reduced = modulus / g;
        y[i] = reduced == 1
                   ? BigInt(0)
                   : residue(integer_matrix_detail::inverse(s.diagonal(i, i) / g, reduced) * (c[i] / g),
                             reduced);
        orders[i] = g;
        steps[i] = reduced;
    }
    for (std::size_t i = s.rank; i < a.rows(); ++i)
        if (residue(c[i], modulus) != 0)
            return {};
    CongruenceLinearSolution out{true, matrix_vector_multiply(s.right, y), {}, {}};
    for (auto &x : out.particular)
        x = residue(x, modulus);
    for (std::size_t j = 0; j < a.cols(); ++j)
        if (orders[j] > 1) {
            std::vector<BigInt> v(a.cols());
            for (std::size_t i = 0; i < a.cols(); ++i)
                v[i] = residue(s.right(i, j) * steps[j], modulus);
            out.generators.push_back(std::move(v));
            out.orders.push_back(orders[j]);
        }
    return out;
}
} // namespace cp
