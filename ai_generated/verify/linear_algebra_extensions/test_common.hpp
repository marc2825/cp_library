#pragma once
#include "basic_utility/big_integer.hpp"
#include "linear_algebra/matrix.hpp"
#include <algorithm>
#include <iostream>
#include <numeric>
#include <random>
#include <set>
#include <stdexcept>
#include <string>
#define CHECK(...)                                                                                           \
    do {                                                                                                     \
        if (!(__VA_ARGS__))                                                                                  \
            throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__) +                \
                                     ": " #__VA_ARGS__);                                                     \
    } while (false)
inline std::mt19937_64 l_random(20261027);
template <class E, class F> void throws(F f) {
    bool caught = false;
    try {
        f();
    } catch (const E &) {
        caught = true;
    }
    CHECK(caught);
}
template <class T> cp::BigInt permutation_determinant(const cp::Matrix<T> &a, bool permanent = false) {
    std::vector<std::size_t> p(a.rows());
    std::iota(p.begin(), p.end(), 0);
    cp::BigInt result = 0;
    do {
        cp::BigInt term = 1;
        bool odd = false;
        for (std::size_t i = 0; i < p.size(); ++i) {
            term *= cp::BigInt(a(i, p[i]));
            for (std::size_t j = 0; j < i; ++j)
                if (p[j] > p[i])
                    odd = !odd;
        }
        result += (odd && !permanent) ? -term : term;
    } while (std::next_permutation(p.begin(), p.end()));
    return result;
}
inline cp::Matrix<int> random_matrix(std::size_t n, std::size_t m, int range = 9) {
    cp::Matrix<int> a(n, m);
    for (std::size_t i = 0; i < n; ++i)
        for (std::size_t j = 0; j < m; ++j)
            a(i, j) = int(l_random() % range) - range / 2;
    return a;
}
inline std::vector<std::size_t> mask_indices(std::size_t mask) {
    std::vector<std::size_t> x;
    for (std::size_t i = 0; mask; ++i, mask >>= 1)
        if (mask & 1)
            x.push_back(i);
    return x;
}
template <int Mod> cp::Matrix<int> modular_product(const cp::Matrix<int> &a, const cp::Matrix<int> &b) {
    cp::Matrix<int> c(a.rows(), b.cols());
    for (std::size_t i = 0; i < a.rows(); ++i)
        for (std::size_t j = 0; j < b.cols(); ++j)
            for (std::size_t k = 0; k < a.cols(); ++k) {
                long long x = (c(i, j) + 1LL * a(i, k) * b(k, j)) % Mod;
                c(i, j) = int(x < 0 ? x + Mod : x);
            }
    return c;
}
