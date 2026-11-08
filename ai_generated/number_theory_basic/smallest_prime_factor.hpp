#pragma once
#include <cstddef>
#include <stdexcept>
#include <utility>
#include <vector>

namespace cp {
// 線形篩で [0,limit] の SPF を構築。0,1 の表の値はそれぞれ0,1。
class SmallestPrimeFactor {
    int limit_;
    std::vector<int> table_, primes_;
public:
    explicit SmallestPrimeFactor(int limit = 0) : limit_(limit) {
        if (limit < 0) throw std::invalid_argument("negative SPF limit");
        table_.assign(static_cast<std::size_t>(limit) + 1, 0);
        if (limit >= 1) table_[1] = 1;
        for (long long i = 2; i <= limit; ++i) {
            if (!table_[i]) { table_[i] = static_cast<int>(i); primes_.push_back(static_cast<int>(i)); }
            for (int p : primes_) {
                if (p > table_[i] || p > limit / i) break;
                table_[i * p] = p;
            }
        }
    }
    int limit() const { return limit_; }
    int operator[](int n) const {
        if (n < 0 || n > limit_) throw std::out_of_range("outside SPF table");
        return table_[n];
    }
    bool is_prime(int n) const { const int p = (*this)[n]; return n >= 2 && p == n; }
    const std::vector<int>& table() const { return table_; }
    const std::vector<int>& primes() const { return primes_; }
    std::vector<std::pair<long long, int>> factorize(int n) const {
        if (n <= 0) throw std::invalid_argument("factorization requires positive n");
        if (n > limit_) throw std::out_of_range("outside SPF table");
        std::vector<std::pair<long long, int>> result;
        while (n > 1) {
            const int p = table_[n]; int exponent = 0;
            do { n /= p; ++exponent; } while (n > 1 && table_[n] == p);
            result.emplace_back(p, exponent);
        }
        return result;
    }
};
} // namespace cp
