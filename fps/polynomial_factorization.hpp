#pragma once
#include "detail/finite_field_polynomial.hpp"
#include <cstdint>
#include <functional>
namespace cp {
struct PolynomialFactor {
    std::vector<int> polynomial;
    std::size_t multiplicity;
};
struct PolynomialFactorization {
    int leading_coefficient;
    std::vector<PolynomialFactor> factors;
};
// Nonzero input only. Factors monic; p-th powers handled even when derivative is zero.
template <int Mod = 998244353>
std::vector<PolynomialFactor> polynomial_square_free_factorization(const std::vector<int> &input) {
    using A = finite_polynomial_detail::Algebra<Mod>;
    using P = std::vector<int>;
    auto f = A::monic(input);
    if (f.empty())
        throw std::domain_error("zero polynomial has no finite factorization");
    std::vector<PolynomialFactor> out;
    std::function<void(P, std::size_t)> solve = [&](P a, std::size_t weight) {
        if (a.size() <= 1)
            return;
        auto c = A::gcd(a, polynomial_derivative<Mod>(a));
        auto w = A::divmod(a, c).first;
        for (std::size_t i = 1; w.size() > 1; ++i) {
            auto y = A::gcd(w, c);
            auto z = A::divmod(w, y).first;
            if (z.size() > 1)
                out.push_back({std::move(z), i * weight});
            w = std::move(y);
            c = A::divmod(c, w).first;
        }
        if (c.size() > 1) {
            P root((c.size() - 1) / Mod + 1);
            for (std::size_t i = 0; i < root.size(); ++i)
                root[i] = c[i * Mod];
            solve(std::move(root), weight * Mod);
        }
    };
    solve(f, 1);
    std::sort(out.begin(), out.end(),
              [](const auto &a, const auto &b) { return a.multiplicity < b.multiplicity; });
    return out;
}
// Input must be monic and square-free. Pair: product, irreducible degree.
template <int Mod = 998244353>
std::vector<std::pair<std::vector<int>, std::size_t>>
polynomial_distinct_degree_factorization(const std::vector<int> &input) {
    using A = finite_polynomial_detail::Algebra<Mod>;
    auto f = A::trim(input);
    if (f.empty() || f.back() != 1 || A::gcd(f, polynomial_derivative<Mod>(f)).size() > 1)
        throw std::invalid_argument("distinct-degree input must be monic square-free");
    std::vector<std::pair<std::vector<int>, std::size_t>> out;
    std::vector<int> h = A::mod({0, 1}, f);
    for (std::size_t d = 1; d <= (f.size() - 1) / 2; ++d) {
        h = A::power(h, Mod, f);
        auto g = A::gcd(f, A::sub(h, {0, 1}));
        if (g.size() > 1) {
            out.push_back({g, d});
            f = A::divmod(f, g).first;
            h = A::mod(h, f);
        }
    }
    if (f.size() > 1)
        out.push_back({f, f.size() - 1});
    return out;
}
namespace finite_polynomial_detail {
inline std::uint64_t random_word(std::uint64_t &state) {
    auto z = (state += UINT64_C(0x9e3779b97f4a7c15));
    z = (z ^ (z >> 30)) * UINT64_C(0xbf58476d1ce4e5b9);
    z = (z ^ (z >> 27)) * UINT64_C(0x94d049bb133111eb);
    return z ^ (z >> 31);
}
template <int Mod>
void split_equal_degree(const std::vector<int> &f, std::size_t d, std::uint64_t &seed,
                        std::vector<std::vector<int>> &out) {
    using A = Algebra<Mod>;
    using P = std::vector<int>;
    if (f.size() == d + 1) {
        out.push_back(f);
        return;
    }
    for (;;) {
        P a(f.size() - 1);
        for (auto &x : a)
            x = static_cast<int>(random_word(seed) % Mod);
        P g = A::gcd(a, f);
        if (g.size() <= 1 || g.size() == f.size()) {
            if constexpr (Mod == 2) {
                P trace{}, v = A::mod(a, f);
                for (std::size_t i = 0; i < d; ++i) {
                    trace = A::add(trace, v);
                    v = A::power(v, 2, f);
                }
                g = A::gcd(trace, f);
            } else {
                P v = A::power(a, (Mod - 1) / 2, f), prod{1};
                for (std::size_t i = 0; i < d; ++i) {
                    prod = A::mod(A::mul(prod, v), f);
                    if (i + 1 < d)
                        v = A::power(v, Mod, f);
                }
                g = A::gcd(A::sub(prod, {1}), f);
            }
        }
        if (g.size() > 1 && g.size() < f.size()) {
            split_equal_degree<Mod>(g, d, seed, out);
            split_equal_degree<Mod>(A::divmod(f, g).first, d, seed, out);
            return;
        }
    }
}
} // namespace finite_polynomial_detail
// Las Vegas Cantor-Zassenhaus. Reproducible seed; no deterministic time guarantee.
template <int Mod = 998244353>
PolynomialFactorization polynomial_factorization(const std::vector<int> &input, std::uint64_t seed = 0) {
    using A = finite_polynomial_detail::Algebra<Mod>;
    auto f = A::trim(input);
    if (f.empty())
        throw std::domain_error("zero polynomial has no finite factorization");
    PolynomialFactorization result{f.back(), {}};
    for (const auto &sf : polynomial_square_free_factorization<Mod>(f))
        for (const auto &dd : polynomial_distinct_degree_factorization<Mod>(sf.polynomial)) {
            std::vector<std::vector<int>> parts;
            finite_polynomial_detail::split_equal_degree<Mod>(dd.first, dd.second, seed, parts);
            for (auto &p : parts)
                result.factors.push_back({std::move(p), sf.multiplicity});
        }
    std::sort(result.factors.begin(), result.factors.end(), [](const auto &a, const auto &b) {
        return a.polynomial.size() != b.polynomial.size() ? a.polynomial.size() < b.polynomial.size()
                                                          : a.polynomial < b.polynomial;
    });
    return result;
}
} // namespace cp
