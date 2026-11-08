#pragma once
#include <cstddef>
#include <stdexcept>
#include <vector>

namespace cp {
// [0,limit] のエラトステネスの篩。limit=0,1 も可。
class PrimeSieve {
    int limit_;
    std::vector<bool> table_;
    std::vector<int> primes_;
public:
    explicit PrimeSieve(int limit = 0) : limit_(limit) {
        if (limit < 0) throw std::invalid_argument("negative sieve limit");
        table_.assign(static_cast<std::size_t>(limit) + 1, true);
        table_[0] = false;
        if (limit >= 1) table_[1] = false;
        for (long long p = 2; p <= limit / p; ++p) if (table_[p])
            for (long long j = p * p; j <= limit; j += p) table_[j] = false;
        for (long long p = 2; p <= limit; ++p) if (table_[p]) primes_.push_back(static_cast<int>(p));
    }
    int limit() const { return limit_; }
    bool is_prime(int n) const {
        if (n < 0 || n > limit_) throw std::out_of_range("outside sieve");
        return table_[n];
    }
    const std::vector<bool>& table() const { return table_; }
    const std::vector<int>& primes() const { return primes_; }
};
} // namespace cp
