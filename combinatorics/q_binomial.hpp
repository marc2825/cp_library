#pragma once
// 素数法の q 階乗と q-Lucas。説明: docs/combinatorics/q_binomial.md
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <vector>
#include "../number_theory_basic/lucas.hpp"
#include "../number_theory_basic/multiplicative_order.hpp"

namespace cp {
class QBinomial {
    int p_, q_, order_;
    LucasBinomial lucas_;
    std::vector<long long> fac_, ifac_;
    static int checked(int p, std::size_t limit) {
        if (p < 2 || !is_prime_64(static_cast<std::uint64_t>(p)))
            throw std::invalid_argument("QBinomial needs a prime modulus");
        if (static_cast<std::size_t>(p) > limit)
            throw std::length_error("QBinomial prime exceeds table budget");
        return p;
    }
public:
    explicit QBinomial(int prime, long long q, std::size_t max_prime = 2000000)
        : p_(checked(prime, max_prime)), q_(static_cast<int>((q % p_ + p_) % p_)),
          order_(q_ == 0 ? 0 : q_ == 1 ? p_ : static_cast<int>(*multiplicative_order(q_, p_))),
          lucas_(p_), fac_(order_ ? order_ : 1, 1), ifac_(fac_.size(), 1) {
        long long power = 1, sum = 0;
        for (int i = 1; i < order_; ++i) {
            sum = (sum + power) % p_;
            power = power * q_ % p_;
            fac_[i] = fac_[i - 1] * sum % p_;
        }
        ifac_.back() = mod_pow(fac_.back(), p_ - 2, p_);
        power = 1; sum = 0;
        std::vector<long long> integer(fac_.size());
        for (int i = 1; i < order_; ++i) {
            sum = (sum + power) % p_; power = power * q_ % p_; integer[i] = sum;
        }
        for (int i = order_ - 1; i > 0; --i) ifac_[i - 1] = ifac_[i] * integer[i] % p_;
    }
    int modulus() const { return p_; }
    int q() const { return q_; }
    // [d]_q が初めて 0 になる d。q=0 なら 0 (存在しない)。
    int zero_period() const { return order_; }
    long long factorial(std::uint64_t n) const {
        if (!q_) return 1;
        return n >= static_cast<std::uint64_t>(order_) ? 0 : fac_[n];
    }
    long long operator()(std::uint64_t n, std::uint64_t k) const {
        if (k > n) return 0;
        if (!q_) return 1;
        if (q_ == 1) return lucas_(n, k);
        const auto a = n % order_, b = k % order_;
        if (b > a) return 0;
        return lucas_(n / order_, k / order_) * fac_[a] % p_ * ifac_[b] % p_ * ifac_[a - b] % p_;
    }
};
} // namespace cp
