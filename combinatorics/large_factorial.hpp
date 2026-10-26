#pragma once
#include <algorithm>
#include <cstdint>
#include <vector>
#include "../convolution/convolution_mod.hpp"
#include "../number_theory_basic/detail/advanced_arithmetic.hpp"
#include "../number_theory_basic/integer_kth_root.hpp"

namespace cp { namespace large_factorial_detail {
// Runtime-prime counterpart of the existing static-field product/remainder tree.
class Polynomial {
    int modulus_;
public:
    using V = std::vector<int>;
    explicit Polynomial(int modulus) : modulus_(modulus) {}
    int multiply(int a, int b) const { return static_cast<int>(static_cast<long long>(a) * b % modulus_); }
    V product(const V& a, const V& b) const {
        if (a.empty() || b.empty()) return {};
        if (std::min(a.size(), b.size()) > 32) return convolution_mod(a, b, modulus_);
        V c(a.size() + b.size() - 1);
        for (std::size_t i = 0; i < a.size(); ++i) for (std::size_t j = 0; j < b.size(); ++j)
            c[i + j] = static_cast<int>((c[i + j] + static_cast<long long>(a[i]) * b[j]) % modulus_);
        return c;
    }
    V inverse(const V& a, std::size_t n) const {
        V result{static_cast<int>(nt_detail::inverse(a[0], modulus_))};
        while (result.size() < n) {
            const auto length = std::min(n, result.size() * 2);
            V prefix(a.begin(), a.begin() + std::min(a.size(), length));
            auto correction = product(prefix, result); correction.resize(length);
            for (auto& value : correction) value = value ? modulus_ - value : 0;
            correction[0] = static_cast<int>((static_cast<long long>(correction[0]) + 2) % modulus_);
            result = product(result, correction); result.resize(length);
        }
        return result;
    }
    V remainder(V a, const V& monic) const {
        if (a.size() < monic.size()) return a;
        const auto length = a.size() - monic.size() + 1;
        if (std::min(length, monic.size()) <= 32) {
            for (std::size_t i = length; i-- > 0;) {
                const auto q = a[i + monic.size() - 1];
                for (std::size_t j = 0; j < monic.size(); ++j) {
                    const auto v = multiply(q, monic[j]);
                    a[i + j] = a[i + j] >= v ? a[i + j] - v : a[i + j] + (modulus_ - v);
                }
            }
        } else {
            V reverse_a(a.rbegin(), a.rbegin() + length), reverse_b(monic.rbegin(), monic.rend());
            auto quotient = product(reverse_a, inverse(reverse_b, length)); quotient.resize(length);
            std::reverse(quotient.begin(), quotient.end());
            const auto sub = product(quotient, monic);
            for (std::size_t i = 0; i < a.size(); ++i) a[i] = a[i] >= sub[i] ? a[i] - sub[i] : a[i] + (modulus_ - sub[i]);
        }
        a.resize(monic.size() - 1);
        return a;
    }
    std::vector<V> tree(const V& roots) const {
        std::size_t base = 1;
        while (base < roots.size()) base *= 2;
        std::vector<V> nodes(base * 2, V{1});
        for (std::size_t i = 0; i < roots.size(); ++i) nodes[base + i] = {roots[i] ? modulus_ - roots[i] : 0, 1};
        for (auto i = base; --i > 0;) nodes[i] = product(nodes[i * 2], nodes[i * 2 + 1]);
        return nodes;
    }
    void evaluate(std::size_t node, V value, const std::vector<V>& nodes, V& output) const {
        const auto base = nodes.size() / 2;
        value = remainder(std::move(value), nodes[node]);
        if (node >= base) { if (node - base < output.size()) output[node - base] = value.empty() ? 0 : value[0]; return; }
        evaluate(node * 2, value, nodes, output);
        if (nodes[node * 2 + 1].size() > 1) evaluate(node * 2 + 1, std::move(value), nodes, output);
    }
};
} // namespace large_factorial_detail
// Prime 2..2e9, runtime modulus. Product/evaluation trees use O(sqrt(n) log n) storage.
inline int large_factorial_mod(std::uint64_t n, int prime, std::uint64_t max_block = 1000000) {
    if (prime < 2 || prime > 2000000000 || !is_prime_64(static_cast<std::uint64_t>(prime)))
        throw std::invalid_argument("factorial requires prime in [2,2e9]");
    if (n >= static_cast<std::uint64_t>(prime)) return 0;
    const bool complement = n > (static_cast<std::uint64_t>(prime) - 1) / 2;
    if (complement) n = static_cast<std::uint64_t>(prime) - 1 - n;
    int result = 1;
    if (n != 0) {
        const auto block = integer_kth_root(n, 2);
        if (block > max_block) throw std::length_error("factorial block budget");
        large_factorial_detail::Polynomial algebra(prime);
        std::vector<int> roots(block), points(n / block);
        for (std::size_t i = 0; i < roots.size(); ++i) roots[i] = prime - static_cast<int>(i + 1);
        for (std::size_t i = 0; i < points.size(); ++i) points[i] = static_cast<int>(i * block);
        const auto polynomial = algebra.tree(roots)[1];
        const auto tree = algebra.tree(points);
        std::vector<int> values(points.size()); algebra.evaluate(1, polynomial, tree, values);
        for (auto value : values) result = algebra.multiply(result, value);
        for (auto i = points.size() * block + 1; i <= n; ++i) result = algebra.multiply(result, static_cast<int>(i));
    }
    if (complement) {
        result = static_cast<int>(nt_detail::inverse(result, prime));
        if (n % 2 == 0) result = prime - result;
    }
    return result;
}
} // namespace cp
