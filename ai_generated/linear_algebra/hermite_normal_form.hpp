#pragma once
#include "detail/integer_matrix.hpp"

namespace cp {
struct HermiteNormalForm {
    Matrix<BigInt> form, left; // form=left*input, row convention, zero rows retained at bottom
    std::vector<std::size_t> pivot_columns;
};
template <class T> HermiteNormalForm hermite_normal_form(const Matrix<T> &input) {
    auto a = integer_matrix_detail::convert(input), u = Matrix<BigInt>::identity(input.rows());
    const auto m = a.rows(), n = a.cols();
    auto swap_rows = [&](std::size_t i, std::size_t j) {
        for (std::size_t c = 0; c < n; ++c)
            std::swap(a(i, c), a(j, c));
        for (std::size_t c = 0; c < m; ++c)
            std::swap(u(i, c), u(j, c));
    };
    auto subtract_row = [&](std::size_t i, std::size_t j, const BigInt &q) {
        for (std::size_t c = 0; c < n; ++c)
            a(i, c) -= q * a(j, c);
        for (std::size_t c = 0; c < m; ++c)
            u(i, c) -= q * u(j, c);
    };
    std::vector<std::size_t> pivots;
    for (std::size_t c = 0, r = 0; c < n && r < m; ++c) {
        auto p = r;
        while (p < m && a(p, c) == 0)
            ++p;
        if (p == m)
            continue;
        swap_rows(r, p);
        for (std::size_t i = r + 1; i < m; ++i)
            while (a(i, c) != 0) {
                BigInt q = a(r, c) / a(i, c);
                subtract_row(r, i, q);
                swap_rows(r, i);
            }
        if (a(r, c) < 0) {
            for (std::size_t j = 0; j < n; ++j)
                a(r, j) = -a(r, j);
            for (std::size_t j = 0; j < m; ++j)
                u(r, j) = -u(r, j);
        }
        for (std::size_t i = 0; i < r; ++i) {
            BigInt q = a(i, c) / a(r, c);
            if (a(i, c) % a(r, c) < 0)
                --q;
            subtract_row(i, r, q);
        }
        pivots.push_back(c);
        ++r;
    }
    return {std::move(a), std::move(u), std::move(pivots)};
}
} // namespace cp
