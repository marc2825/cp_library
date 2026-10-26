#pragma once
// O(p^2) 前計算、uint64 引数の第1/第2種を O(log_p n)。
// 説明: docs/combinatorics/stirling_small_prime.md
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <vector>
#include "../number_theory_basic/lucas.hpp"
namespace cp {
class StirlingSmallPrime {
    int p_;
    LucasBinomial binomial_;
    std::vector<std::vector<int>> first_, periodic_;
    static int checked(int p, std::size_t cells) {
        if (p < 2 || !is_prime_64(static_cast<std::uint64_t>(p)))
            throw std::invalid_argument("Stirling modulus must be prime");
        if (static_cast<std::size_t>(p) > cells / 2 / static_cast<std::size_t>(p))
            throw std::length_error("Stirling table exceeds cell budget");
        return p;
    }
public:
    explicit StirlingSmallPrime(int prime, std::size_t max_cells = 4000000)
        : p_(checked(prime, max_cells)), binomial_(p_),
          first_(p_, std::vector<int>(p_)), periodic_(p_, std::vector<int>(p_ - 1)) {
        first_[0][0] = 1;
        for (int n = 1; n < p_; ++n) for (int k = 1; k <= n; ++k)
            first_[n][k] = static_cast<int>((first_[n - 1][k - 1] +
                static_cast<long long>(n - 1) * first_[n - 1][k]) % p_);
        periodic_[0][0] = 1;
        for (int r = 1; r < p_; ++r) {
            periodic_[r][0] = 1;
            for (int t = 1; t < p_ - 1; ++t)
                periodic_[r][t] = static_cast<int>((periodic_[r - 1][t] +
                    static_cast<long long>(r) * periodic_[r][t - 1]) % p_);
        }
    }
    int modulus() const { return p_; }
    int first(std::uint64_t n, std::uint64_t k, bool signed_values = false) const {
        if (k > n) return 0;
        const auto q = n / p_, r = n % p_, d = static_cast<std::uint64_t>(p_ - 1);
        if (k < q) return 0;
        const auto i = (k - q) % d, j = (k - q) / d;
        auto term = [&](std::uint64_t degree, std::uint64_t index) -> long long {
            if (degree > r || index > q) return 0;
            const auto value = first_[r][degree] * binomial_(q, index) % p_;
            return ((q - index) & 1) && value ? p_ - value : value;
        };
        auto answer = term(i, j);
        if (j && i + d <= r) answer = (answer + term(i + d, j - 1)) % p_;
        if (signed_values && ((n - k) & 1) && answer) answer = p_ - answer;
        return static_cast<int>(answer);
    }
    int second(std::uint64_t n, std::uint64_t k) const {
        if (k > n) return 0;
        if (!k) return n == 0;
        const auto q = k / p_, r = k % p_, t = n - k;
        const auto u = t / (p_ - 1), v = t % (p_ - 1);
        if (!r) return v ? 0 : static_cast<int>(binomial_(q + u - 1, u));
        return static_cast<int>(periodic_[r][v] * binomial_(q + u, u) % p_);
    }
};
} // namespace cp
