#pragma once
#include <cstdint>
#include <stdexcept>
#include <utility>
#include <vector>
#include "integer_kth_root.hpp"

namespace cp {
// [left,right) の素数と素因数分解。0 の分解は domain_error、1 は空。
class SegmentedSieve {
    std::uint64_t left_, right_;
    std::vector<std::vector<std::pair<std::uint64_t, int>>> factors_;
    std::vector<std::uint64_t> primes_;
public:
    SegmentedSieve(std::uint64_t left, std::uint64_t right,
                   std::uint64_t max_width = 1000000, std::uint64_t max_root = 5000000)
        : left_(left), right_(right) {
        if (left > right) throw std::invalid_argument("reversed sieve interval");
        if (right - left > max_width || right - left > factors_.max_size())
            throw std::length_error("segmented sieve width budget");
        if (left == right) return;
        const auto root = integer_kth_root(right - 1, 2);
        if (root > max_root || root >= std::vector<bool>().max_size())
            throw std::length_error("segmented sieve root budget");
        std::vector<bool> composite(static_cast<std::size_t>(root) + 1);
        factors_.resize(static_cast<std::size_t>(right - left));
        std::vector<std::uint64_t> remaining(factors_.size());
        for (std::size_t i = 0; i < remaining.size(); ++i) remaining[i] = left + i;
        for (std::uint64_t p = 2; p <= root; ++p) if (!composite[p]) {
            if (p <= root / p) for (auto j = p * p; j <= root; j += p) composite[j] = true;
            const auto offset = (p - left % p) % p;
            for (auto j = offset; j < right - left;) {
                auto& value = remaining[j];
                if (value > 1 && value % p == 0) {
                    int exponent = 0;
                    do { value /= p; ++exponent; } while (value % p == 0);
                    factors_[j].emplace_back(p, exponent);
                }
                if (p >= right - left - j) break;
                j += p;
            }
        }
        for (std::size_t i = 0; i < remaining.size(); ++i) {
            if (remaining[i] > 1) factors_[i].emplace_back(remaining[i], 1);
            if (factors_[i].size() == 1 && factors_[i][0] == std::make_pair(left + i, 1))
                primes_.push_back(left + i);
        }
    }
    const auto& primes() const { return primes_; }
    const auto& factorization(std::uint64_t x) const {
        if (x < left_ || x >= right_) throw std::out_of_range("segmented sieve query");
        if (!x) throw std::domain_error("zero factorization");
        return factors_[static_cast<std::size_t>(x - left_)];
    }
};
} // namespace cp
