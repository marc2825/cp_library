#pragma once
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <vector>
#include "../basic_utility/integer_kth_root.hpp"
#include "detail/arithmetic.hpp"

namespace cp {
// F(x)=sum_{i<=x}f(i), only at x=floor(N/j). F(0)=0, no interpolation.
template<class T> class QuotientPrefix {
    std::uint64_t n_, root_;
    std::vector<std::uint64_t> quotients_;
    std::vector<T> data_;
    std::size_t index(std::uint64_t x) const {
        if (!x || x > n_ || n_ / (n_ / x) != x) throw std::invalid_argument("not a positive floor quotient");
        return x <= root_ ? data_.size() - static_cast<std::size_t>(x) : static_cast<std::size_t>(n_ / x - 1);
    }
public:
    template<class Prefix> QuotientPrefix(std::uint64_t n, Prefix prefix, std::uint64_t max_root = 5000000)
        : n_(n), root_(integer_kth_root(n, 2)) {
        if (root_ > max_root || root_ > (data_.max_size() - 1) / 2) throw std::length_error("quotient table root budget");
        for (std::uint64_t left = 1; left <= n;) {
            const auto q = n / left, right = n / q;
            quotients_.push_back(q); data_.push_back(prefix(q));
            if (right == n) break;
            left = right + 1;
        }
    }
    std::uint64_t limit() const { return n_; }
    std::uint64_t root() const { return root_; }
    const auto& quotients() const { return quotients_; }
    T sum(std::uint64_t x) const { return x ? data_[index(x)] : T(0); }
    T total() const { return sum(n_); }
    void set(std::uint64_t x, T value) { data_[index(x)] = std::move(value); }
};
namespace quotient_detail {
template<class T> T add(T a, const T& b) { detail::arithmetic_add_assign(a, b); return a; }
template<class T> T sub(T a, const T& b) { detail::arithmetic_subtract_assign(a, b); return a; }
template<class T> T mul(const T& a, const T& b) { return detail::arithmetic_multiply(a, b); }
inline std::vector<std::uint64_t> small_primes(std::uint64_t root) {
    std::vector<bool> composite(static_cast<std::size_t>(root) + 1);
    std::vector<std::uint64_t> primes;
    for (std::uint64_t p = 2; p <= root; ++p) if (!composite[p]) {
        primes.push_back(p);
        if (p <= root / p) for (auto j = p * p; j <= root; j += p) composite[j] = true;
    }
    return primes;
}
} // namespace quotient_detail
} // namespace cp
