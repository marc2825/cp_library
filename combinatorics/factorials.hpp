#pragma once
#include <cstddef>
#include <stdexcept>
#include <vector>
#include "../number_theory_basic/mod_pow.hpp"
#include "../number_theory_basic/mod_inverse.hpp"

namespace cp {
// 0!〜max_n! mod modulus。逆元が不要なら、合成数法でもこの関数だけでよい。
inline std::vector<long long> factorials(int max_n, long long modulus = 998244353) {
    if (max_n < 0 || modulus <= 0) throw std::invalid_argument("invalid factorial bound or modulus");
    std::vector<long long> result(static_cast<std::size_t>(max_n) + 1, 1 % modulus);
    for (long long i = 1; i <= max_n; ++i) result[i] = mod_multiply(result[i - 1], i, modulus);
    return result;
}
// max_n! が法と互いに素である場合の階乗・逆階乗表。上限を含む。
class FactorialTable {
    long long modulus_;
    std::vector<long long> factorial_, inverse_factorial_;
public:
    explicit FactorialTable(int max_n, long long modulus = 998244353)
        : modulus_(modulus), factorial_(cp::factorials(max_n, modulus)) {
        const auto inverse = mod_inverse(factorial_.back(), modulus);
        if (!inverse) throw std::domain_error("max_n! is not invertible; use PascalBinomial for binomial queries");
        inverse_factorial_.resize(factorial_.size());
        inverse_factorial_[max_n] = *inverse;
        for (int i = max_n; i > 0; --i) inverse_factorial_[i - 1] = mod_multiply(inverse_factorial_[i], i, modulus);
    }
    int max_n() const { return static_cast<int>(factorial_.size() - 1); }
    long long modulus() const { return modulus_; }
    long long factorial(long long n) const {
        if (n < 0 || n > max_n()) throw std::out_of_range("factorial query outside table");
        return factorial_[static_cast<std::size_t>(n)];
    }
    long long inverse_factorial(long long n) const {
        if (n < 0 || n > max_n()) throw std::out_of_range("inverse factorial query outside table");
        return inverse_factorial_[static_cast<std::size_t>(n)];
    }
    const std::vector<long long>& factorials() const { return factorial_; }
    const std::vector<long long>& inverse_factorials() const { return inverse_factorial_; }
};
} // namespace cp
