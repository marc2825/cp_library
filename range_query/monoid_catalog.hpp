#pragma once
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include "segtree.hpp"
#include "range_actions.hpp"
#include "../linear_algebra/matrix_product.hpp"

namespace cp {
// Monoid: S, op(S,S), e(). ActedMonoid adds F, mapping, composition, id.
template <class M> using MonoidSegtree = Segtree<typename M::S, M::op, M::e>;
template <class A> using ActedMonoidLazySegtree = ActionLazySegtree<A>;

template <class T, bool Maximum = false> struct ExtremumCountMonoid {
    struct S { T value; std::size_t count; };
    static S e() { return {T{}, 0}; }
    static S leaf(T value) { return {value, 1}; }
    static S op(S a, S b) {
        if (!a.count) return b;
        if (!b.count) return a;
        if (a.value < b.value) return Maximum ? b : a;
        if (b.value < a.value) return Maximum ? a : b;
        return {a.value, a.count + b.count};
    }
};
template <class T> using MinCountMonoid = ExtremumCountMonoid<T, false>;
template <class T> using MaxCountMonoid = ExtremumCountMonoid<T, true>;

template <class T, bool AllowEmpty = false> struct MaxSubarrayMonoid {
    struct S { T sum, prefix, suffix, best; std::size_t length; };
    static S e() { return {T(0), T(0), T(0), T(0), 0}; }
    static S leaf(T value) {
        const T best = AllowEmpty ? std::max(T(0), value) : value;
        return {value, best, best, best, 1};
    }
    static S op(S a, S b) {
        if (!a.length) return b;
        if (!b.length) return a;
        return {a.sum + b.sum, std::max(a.prefix, a.sum + b.prefix),
            std::max(b.suffix, a.suffix + b.sum),
            std::max({a.best, b.best, a.suffix + b.prefix}), a.length + b.length};
    }
};

// Ordered product A_left * A_right, sharing the existing checked matrix product.
template <class T, std::size_t Dimension> struct MatrixProductMonoid {
    using S = Matrix<T>;
    static S e() { return S::identity(Dimension); }
    static S op(S a, S b) {
        if (a.rows() != Dimension || a.cols() != Dimension || b.rows() != Dimension || b.cols() != Dimension)
            throw std::invalid_argument("monoid matrix dimension mismatch");
        return matrix_multiply(a, b);
    }
};

// op(f,g) applies f first and then g, matching left-to-right function queries.
template <class T> struct AffineFunctionMonoid {
    struct S { T a, b; T operator()(T x) const { return a * x + b; } };
    static S e() { return {T(1), T(0)}; }
    static S op(S f, S g) { return {g.a * f.a, g.a * f.b + g.b}; }
};

// Invertible transforms over a field; nullopt denotes the point at infinity.
template <class T> struct LinearFractionalMonoid {
    struct S { T a, b, c, d; };
    static S e() { return {T(1), T(0), T(0), T(1)}; }
    static S make(T a, T b, T c, T d) {
        if (a * d == b * c) throw std::invalid_argument("singular linear fractional transform");
        return {a, b, c, d};
    }
    static S op(S f, S g) {
        return {g.a*f.a + g.b*f.c, g.a*f.b + g.b*f.d,
                g.c*f.a + g.d*f.c, g.c*f.b + g.d*f.d};
    }
    static std::optional<T> apply(S f, std::optional<T> x) {
        const T numerator = x ? f.a * *x + f.b : f.a;
        const T denominator = x ? f.c * *x + f.d : f.c;
        if (denominator == T(0)) {
            if (numerator == T(0)) throw std::domain_error("undefined linear fractional image");
            return std::nullopt;
        }
        return numerator / denominator;
    }
};

template <class Mint, std::uint64_t Base> struct RollingHashMonoid {
    struct S { Mint hash, power; std::size_t length; };
    static S e() { return {Mint(0), Mint(1), 0}; }
    static S leaf(Mint code) { return {code, Mint(Base), 1}; }
    static S op(S a, S b) { return {a.hash * b.power + b.hash, a.power * b.power, a.length + b.length}; }
};
} // namespace cp
