#pragma once
#include "detail/finite_field_polynomial.hpp"
#include "subproduct_tree.hpp"
#include "taylor_shift.hpp"
#include <cmath>
#include <functional>
namespace cp {
using PolynomialMatrix = std::vector<std::vector<std::vector<int>>>;
namespace polynomial_matrix_detail {
inline std::size_t dimension(const PolynomialMatrix &a) {
    auto n = a.size();
    for (const auto &r : a)
        if (r.size() != n)
            throw std::invalid_argument("polynomial matrix must be square");
    return n;
}
template <int Mod> PolynomialMatrix identity(std::size_t n) {
    PolynomialMatrix a(n, std::vector<std::vector<int>>(n));
    for (std::size_t i = 0; i < n; ++i)
        a[i][i] = {1 % Mod};
    return a;
}
template <int Mod, int Root> PolynomialMatrix multiply(const PolynomialMatrix &a, const PolynomialMatrix &b) {
    auto n = a.size();
    PolynomialMatrix c(n, std::vector<std::vector<int>>(n));
    for (std::size_t i = 0; i < n; ++i)
        for (std::size_t j = 0; j < n; ++j)
            for (std::size_t k = 0; k < n; ++k)
                c[i][j] = polynomial_add<Mod>(c[i][j], fps_detail::multiply<Mod, Root>(a[i][k], b[k][j]));
    return c;
}
} // namespace polynomial_matrix_detail
// Fraction-free Bareiss elimination over F_p[x], including singular polynomial matrices.
template <int Mod = 998244353> std::vector<int> polynomial_matrix_determinant(PolynomialMatrix a) {
    using A = finite_polynomial_detail::Algebra<Mod>;
    auto n = polynomial_matrix_detail::dimension(a);
    if (!n)
        return {1};
    for (auto &r : a)
        for (auto &p : r)
            p = A::trim(std::move(p));
    std::vector<int> previous{1};
    int sign = 1;
    for (std::size_t k = 0; k + 1 < n; ++k) {
        auto pivot = k;
        while (pivot < n && a[pivot][k].empty())
            ++pivot;
        if (pivot == n)
            return {};
        if (pivot != k) {
            std::swap(a[k], a[pivot]);
            sign = -sign;
        }
        for (std::size_t i = k + 1; i < n; ++i) {
            for (std::size_t j = k + 1; j < n; ++j) {
                auto numerator = A::sub(A::mul(a[i][j], a[k][k]), A::mul(a[i][k], a[k][j]));
                auto qr = A::divmod(numerator, previous);
                if (!qr.second.empty())
                    throw std::logic_error("nonexact Bareiss division");
                a[i][j] = std::move(qr.first);
            }
            a[i][k].clear();
        }
        previous = a[k][k];
    }
    return sign == 1 ? a.back().back() : A::scale(a.back().back(), Mod - 1);
}
// M(start+count-1)...M(start), evaluated in F_p. Baby-step polynomial blocks + multipoint evaluation.
// max_block_degree bounds construction memory; count=0 returns identity.
template <int Mod = 998244353, int Root = 3>
std::vector<std::vector<int>> polynomial_matrix_prefix_product(const PolynomialMatrix &a, std::uint64_t start,
                                                               std::uint64_t count,
                                                               std::size_t max_block_degree = 65536) {
    using F = fps_detail::Field<Mod>;
    const auto n = polynomial_matrix_detail::dimension(a);
    std::size_t degree = 0;
    for (const auto &r : a)
        for (const auto &p : r) {
            auto t = polynomial_trim<Mod>(p);
            if (!t.empty())
                degree = std::max(degree, t.size() - 1);
        }
    using Matrix = std::vector<std::vector<int>>;
    Matrix result(n, std::vector<int>(n));
    for (std::size_t i = 0; i < n; ++i)
        result[i][i] = 1;
    auto mul = [&](const Matrix &x, const Matrix &y) {
        Matrix z(n, std::vector<int>(n));
        for (std::size_t i = 0; i < n; ++i)
            for (std::size_t j = 0; j < n; ++j)
                for (std::size_t k = 0; k < n; ++k)
                    z[i][j] = F::add(z[i][j], F::mul(x[i][k], y[k][j]));
        return z;
    };
    if (!count || !n)
        return result;
    if (!degree) {
        Matrix base(n, std::vector<int>(n));
        for (std::size_t i = 0; i < n; ++i)
            for (std::size_t j = 0; j < n; ++j)
                base[i][j] = polynomial_evaluate<Mod>(a[i][j], 0);
        for (; count; count >>= 1) {
            if (count & 1)
                result = mul(base, result);
            if (count > 1)
                base = mul(base, base);
        }
        return result;
    }
    const auto block = std::max<std::uint64_t>(
        1, static_cast<std::uint64_t>(std::sqrt(static_cast<long double>(count) / degree)));
    if (block > max_block_degree / degree || block * degree >= static_cast<std::uint64_t>(Mod) ||
        block * degree + 1 > fps_max_size<Mod, Root>() || count / block > max_block_degree)
        throw std::length_error("polynomial matrix block/evaluation budget exceeded");
    std::function<PolynomialMatrix(std::uint64_t, std::uint64_t)> build = [&](auto l,
                                                                              auto r) -> PolynomialMatrix {
        if (r - l == 1) {
            auto b = a;
            for (auto &row : b)
                for (auto &p : row)
                    p = taylor_shift<Mod, Root>(p, static_cast<long long>(l % Mod));
            return b;
        }
        auto m = l + (r - l) / 2;
        return polynomial_matrix_detail::multiply<Mod, Root>(build(m, r), build(l, m));
    };
    auto product = build(0, block);
    const auto blocks = static_cast<std::size_t>(count / block);
    std::vector<int> points(blocks);
    for (std::size_t k = 0; k < blocks; ++k)
        points[k] = static_cast<int>((start % Mod + (std::uint64_t(k) % Mod) * (block % Mod)) % Mod);
    SubproductTree<Mod, Root> tree(points);
    std::vector<std::vector<std::vector<int>>> values(n, std::vector<std::vector<int>>(n));
    for (std::size_t i = 0; i < n; ++i)
        for (std::size_t j = 0; j < n; ++j)
            values[i][j] = tree.evaluate(product[i][j]);
    for (std::size_t k = 0; k < blocks; ++k) {
        Matrix value(n, std::vector<int>(n));
        for (std::size_t i = 0; i < n; ++i)
            for (std::size_t j = 0; j < n; ++j)
                value[i][j] = values[i][j][k];
        result = mul(value, result);
    }
    for (std::uint64_t k = count - count % block; k < count; ++k) {
        Matrix value(n, std::vector<int>(n));
        auto x = (start % Mod + k % Mod) % Mod;
        for (std::size_t i = 0; i < n; ++i)
            for (std::size_t j = 0; j < n; ++j)
                value[i][j] = polynomial_evaluate<Mod>(a[i][j], x);
        result = mul(value, result);
    }
    return result;
}
} // namespace cp
