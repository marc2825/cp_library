#pragma once
#include "../fps/detail/finite_field_polynomial.hpp"
#include "detail/smith_reduction.hpp"
#include "inverse.hpp"

namespace cp {
struct FrobeniusNormalForm {
    Matrix<int> form, basis, inverse_basis;          // A*basis=basis*form
    std::vector<std::vector<int>> invariant_factors; // monic, positive degree, successive divisibility
};
namespace frobenius_detail {
template <int Mod> struct Ring {
    using A = finite_polynomial_detail::Algebra<Mod>;
    using value_type = std::vector<int>;
    static value_type zero() { return {}; }
    static value_type one() { return {1}; }
    static auto add(const value_type &a, const value_type &b) { return A::add(a, b); }
    static auto sub(const value_type &a, const value_type &b) { return A::sub(a, b); }
    static auto mul(const value_type &a, const value_type &b) { return A::mul(a, b); }
    static auto divmod(const value_type &a, const value_type &b) { return A::divmod(a, b); }
    static value_type normalizing_unit(const value_type &a) { return {A::F::inv(a.back())}; }
    static value_type inverse_unit(const value_type &a) { return {A::F::inv(a[0])}; }
};
} // namespace frobenius_detail
// Polynomial Smith form of xI-A, with module generators mapped back through inverse row operations.
template <int Mod = 998244353> FrobeniusNormalForm frobenius_normal_form(const Matrix<int> &input) {
    using R = frobenius_detail::Ring<Mod>;
    using F = typename R::A::F;
    using P = std::vector<int>;
    const auto n = input.rows();
    if (n != input.cols())
        throw std::invalid_argument("Frobenius form needs a square matrix");
    Matrix<int> a(n, n);
    Matrix<P> pencil(n, n, P{});
    for (std::size_t i = 0; i < n; ++i)
        for (std::size_t j = 0; j < n; ++j) {
            a(i, j) = F::norm(input(i, j));
            pencil(i, j) = R::A::trim({F::sub(0, a(i, j))});
            if (i == j) {
                pencil(i, j).resize(2);
                pencil(i, j)[1] = 1;
            }
        }
    auto s = normal_form_detail::smith<R, true>(std::move(pencil));
    FrobeniusNormalForm out{Matrix<int>(n, n), Matrix<int>(n, n), Matrix<int>(), {}};
    auto apply = [&](const std::vector<int> &v) {
        std::vector<int> w(n);
        for (std::size_t i = 0; i < n; ++i)
            for (std::size_t j = 0; j < n; ++j)
                w[i] = F::add(w[i], F::mul(a(i, j), v[j]));
        return w;
    };
    std::size_t offset = 0;
    for (std::size_t c = 0; c < n; ++c) {
        const auto &f = s.diagonal(c, c);
        if (f.size() < 2)
            continue;
        const auto d = f.size() - 1;
        out.invariant_factors.push_back(f);
        std::size_t length = 0;
        for (std::size_t i = 0; i < n; ++i)
            length = std::max(length, s.inverse_left(i, c).size());
        std::vector<int> v(n);
        for (std::size_t k = length; k-- > 0;) {
            v = apply(v);
            for (std::size_t i = 0; i < n; ++i)
                if (k < s.inverse_left(i, c).size())
                    v[i] = F::add(v[i], s.inverse_left(i, c)[k]);
        }
        for (std::size_t j = 0; j < d; ++j) {
            for (std::size_t i = 0; i < n; ++i)
                out.basis(i, offset + j) = v[i];
            if (j + 1 < d) {
                out.form(offset + j + 1, offset + j) = 1;
                v = apply(v);
            }
            out.form(offset + j, offset + d - 1) = F::sub(0, f[j]);
        }
        offset += d;
    }
    auto inverse = matrix_inverse_mod<Mod>(out.basis);
    if (offset != n || !inverse)
        throw std::logic_error("Frobenius cyclic basis invariant failed");
    out.inverse_basis = std::move(*inverse);
    return out;
}
} // namespace cp
