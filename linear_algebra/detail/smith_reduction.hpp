#pragma once
#include "../matrix.hpp"

namespace cp {
namespace normal_form_detail {
template <class R> Matrix<typename R::value_type> identity(std::size_t n) {
    Matrix<typename R::value_type> a(n, n, R::zero());
    for (std::size_t i = 0; i < n; ++i)
        a(i, i) = R::one();
    return a;
}
template <class T> struct SmithReduction {
    Matrix<T> diagonal, left, right, inverse_left;
    std::size_t rank = 0;
};
// Euclidean-domain reduction. Each completed pivot divides the entire remaining block.
template <class R, bool TrackInverse = false>
SmithReduction<typename R::value_type> smith(Matrix<typename R::value_type> a) {
    using T = typename R::value_type;
    auto m = a.rows(), n = a.cols();
    auto u = identity<R>(m), v = identity<R>(n);
    Matrix<T> w = TrackInverse ? identity<R>(m) : Matrix<T>();
    auto row_add = [&](std::size_t i, std::size_t j, const T &factor) {
        for (std::size_t c = 0; c < n; ++c)
            a(i, c) = R::add(a(i, c), R::mul(factor, a(j, c)));
        for (std::size_t c = 0; c < m; ++c)
            u(i, c) = R::add(u(i, c), R::mul(factor, u(j, c)));
        if constexpr (TrackInverse)
            for (std::size_t r = 0; r < m; ++r)
                w(r, j) = R::sub(w(r, j), R::mul(factor, w(r, i)));
    };
    auto row_swap = [&](std::size_t i, std::size_t j) {
        for (std::size_t c = 0; c < n; ++c)
            std::swap(a(i, c), a(j, c));
        for (std::size_t c = 0; c < m; ++c)
            std::swap(u(i, c), u(j, c));
        if constexpr (TrackInverse)
            for (std::size_t r = 0; r < m; ++r)
                std::swap(w(r, i), w(r, j));
    };
    auto col_add = [&](std::size_t i, std::size_t j, const T &factor) {
        for (std::size_t r = 0; r < m; ++r)
            a(r, i) = R::add(a(r, i), R::mul(factor, a(r, j)));
        for (std::size_t r = 0; r < n; ++r)
            v(r, i) = R::add(v(r, i), R::mul(factor, v(r, j)));
    };
    auto col_swap = [&](std::size_t i, std::size_t j) {
        for (std::size_t r = 0; r < m; ++r)
            std::swap(a(r, i), a(r, j));
        for (std::size_t r = 0; r < n; ++r)
            std::swap(v(r, i), v(r, j));
    };
    std::size_t rank = 0;
    while (rank < std::min(m, n)) {
        const auto k = rank;
        std::size_t pr = m, pc = n;
        for (std::size_t i = k; i < m && pr == m; ++i)
            for (std::size_t j = k; j < n; ++j)
                if (a(i, j) != R::zero()) {
                    pr = i;
                    pc = j;
                    break;
                }
        if (pr == m)
            break;
        row_swap(k, pr);
        col_swap(k, pc);
        for (;;) {
            for (std::size_t i = k + 1; i < m; ++i)
                while (a(i, k) != R::zero()) {
                    T q = R::divmod(a(i, k), a(k, k)).first;
                    row_add(i, k, R::sub(R::zero(), q));
                    if (a(i, k) != R::zero())
                        row_swap(k, i);
                }
            for (std::size_t j = k + 1; j < n; ++j)
                while (a(k, j) != R::zero()) {
                    T q = R::divmod(a(k, j), a(k, k)).first;
                    col_add(j, k, R::sub(R::zero(), q));
                    if (a(k, j) != R::zero())
                        col_swap(k, j);
                }
            bool clear = true;
            for (std::size_t i = k + 1; i < m; ++i)
                if (a(i, k) != R::zero())
                    clear = false;
            if (!clear)
                continue;
            std::size_t bad = m;
            for (std::size_t i = k + 1; i < m && bad == m; ++i)
                for (std::size_t j = k + 1; j < n; ++j)
                    if (R::divmod(a(i, j), a(k, k)).second != R::zero()) {
                        bad = i;
                        break;
                    }
            if (bad == m)
                break;
            row_add(k, bad, R::one());
        }
        const T unit = R::normalizing_unit(a(k, k));
        for (std::size_t j = 0; j < n; ++j)
            a(k, j) = R::mul(unit, a(k, j));
        for (std::size_t j = 0; j < m; ++j)
            u(k, j) = R::mul(unit, u(k, j));
        if constexpr (TrackInverse)
            for (std::size_t i = 0; i < m; ++i)
                w(i, k) = R::mul(R::inverse_unit(unit), w(i, k));
        ++rank;
    }
    return {std::move(a), std::move(u), std::move(v), std::move(w), rank};
}
} // namespace normal_form_detail
} // namespace cp
