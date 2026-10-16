#pragma once
#include <cstddef>
#include <stdexcept>
#include <vector>
#include "factorials.hpp"
#include "detail/arithmetic.hpp"

namespace cp {
// 不正な n,k は0。有効な n が前計算上限を超える場合は out_of_range。
inline long long binomial(const FactorialTable& table, long long n, long long k) {
    if (n < 0 || k < 0 || k > n) return 0;
    return mod_multiply(mod_multiply(table.factorial(n), table.inverse_factorial(k), table.modulus()),
                        table.inverse_factorial(n - k), table.modulus());
}
// 加算だけで構築する O(N^2) の二項係数表。法が合成数、n>=法でも可。
class PascalBinomial {
    long long modulus_;
    std::vector<std::vector<long long>> rows_;
public:
    explicit PascalBinomial(int max_n, long long modulus = 998244353) : modulus_(modulus) {
        if (max_n < 0 || modulus <= 0) throw std::invalid_argument("invalid Pascal bound or modulus");
        rows_.reserve(static_cast<std::size_t>(max_n) + 1);
        for (long long n = 0; n <= max_n; ++n) {
            rows_.emplace_back(static_cast<std::size_t>(n) + 1, 1 % modulus);
            for (long long k = 1; k < n; ++k)
                rows_[n][k] = combinatorics_detail::add_mod(rows_[n - 1][k - 1], rows_[n - 1][k], modulus);
        }
    }
    int max_n() const { return static_cast<int>(rows_.size() - 1); }
    long long modulus() const { return modulus_; }
    long long operator()(long long n, long long k) const {
        if (n < 0 || k < 0 || k > n) return 0;
        if (n > max_n()) throw std::out_of_range("binomial query outside Pascal table");
        return rows_[static_cast<std::size_t>(n)][static_cast<std::size_t>(k)];
    }
    const std::vector<std::vector<long long>>& rows() const { return rows_; }
};
inline long long binomial(const PascalBinomial& table, long long n, long long k) { return table(n, k); }
} // namespace cp
