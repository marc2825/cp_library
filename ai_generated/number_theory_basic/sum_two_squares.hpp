#pragma once
#include <algorithm>
#include <cstdint>
#include <set>
#include <utility>
#include <vector>
#include "gaussian_integer.hpp"
#include "primitive_root.hpp"
#include "../number_theory_convolution/pollard_rho.hpp"

namespace cp {
// All unordered nonnegative pairs x<=y, x*x+y*y=n. Intermediate Gaussian values are exact.
inline std::vector<std::pair<std::uint64_t, std::uint64_t>> sum_two_squares(
        std::uint64_t n, std::size_t max_states = 1000000) {
    if (!max_states) throw std::length_error("two-squares state budget");
    if (n == 0) return {{0, 0}};
    const auto factors = prime_factorization_64(n);
    for (auto [p, e] : factors) if (p % 4 == 3 && e % 2) return {};
    std::vector<GaussianInteger> states{GaussianInteger(1)};
    for (auto [p, e] : factors) {
        if (p % 4 == 3) {
            const GaussianInteger scale(nt_detail::power(BigInt(p), static_cast<unsigned>(e / 2)));
            for (auto& z : states) z = z * scale;
        } else if (p == 2) {
            GaussianInteger factor(1);
            for (int i = 0; i < e; ++i) factor = factor * GaussianInteger(1, 1);
            for (auto& z : states) z = z * factor;
        } else {
            const auto root = nt_detail::pow(primitive_root(p), (p - 1) / 4, p);
            const auto z = gaussian_gcd(GaussianInteger(p), GaussianInteger(root, 1));
            std::vector<GaussianInteger> left(static_cast<std::size_t>(e) + 1, GaussianInteger(1)), right = left;
            for (int i = 1; i <= e; ++i) { left[i] = left[i - 1] * z; right[i] = right[i - 1] * z.conjugate(); }
            if (states.size() > max_states / (static_cast<std::size_t>(e) + 1)) throw std::length_error("two-squares state budget");
            std::vector<GaussianInteger> next;
            for (const auto& value : states) for (int i = 0; i <= e; ++i) next.push_back(value * left[i] * right[e - i]);
            states = std::move(next);
        }
    }
    std::set<std::pair<std::uint64_t, std::uint64_t>> result;
    for (const auto& z : states) {
        auto x = abs(z.real).to_integral<std::uint64_t>(), y = abs(z.imag).to_integral<std::uint64_t>();
        if (x > y) std::swap(x, y);
        result.emplace(x, y);
    }
    return {result.begin(), result.end()};
}
} // namespace cp
