#pragma once
#include <array>
#include <cstddef>
#include <vector>
#include "monoid_catalog.hpp"

namespace cp {
// Add a polynomial in the absolute coordinate. Moments make mapping distributive.
template <class T, std::size_t Degree> struct PolynomialAddSumAction {
    struct S { T sum; std::array<T, Degree + 1> moments; };
    using F = std::array<T, Degree + 1>;
    static F id() { F result; result.fill(T(0)); return result; }
    static S e() { return {T(0), id()}; }
    static S leaf(T value, T coordinate) {
        S result = e(); result.sum = value;
        T power = T(1);
        for (std::size_t k = 0; k <= Degree; ++k) {
            result.moments[k] = power;
            if (k < Degree) power = power * coordinate;
        }
        return result;
    }
    static std::vector<S> values(const std::vector<T>& a) {
        std::vector<S> result; result.reserve(a.size());
        for (std::size_t i = 0; i < a.size(); ++i) result.push_back(leaf(a[i], T(i)));
        return result;
    }
    static S op(S a, S b) {
        a.sum = a.sum + b.sum;
        for (std::size_t k = 0; k <= Degree; ++k) a.moments[k] = a.moments[k] + b.moments[k];
        return a;
    }
    static S mapping(F f, S x) {
        for (std::size_t k = 0; k <= Degree; ++k) x.sum = x.sum + f[k] * x.moments[k];
        return x;
    }
    static F composition(F f, F g) {
        for (std::size_t k = 0; k <= Degree; ++k) f[k] = f[k] + g[k];
        return f;
    }
};
template <class T> struct ArithmeticProgressionAddSumAction : PolynomialAddSumAction<T, 1> {
    using F = typename PolynomialAddSumAction<T, 1>::F;
    // For apply(l,r,...), add first, first+step, ... starting at position l.
    static F progression(std::size_t l, T first, T step) { return {first - step * T(l), step}; }
};
} // namespace cp
