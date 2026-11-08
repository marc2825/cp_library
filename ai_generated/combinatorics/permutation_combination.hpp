#pragma once
#include "binomial.hpp"

namespace cp {
// 異なるn個から順序付きでk個。nPk = n!/(n-k)!。
inline long long permutations(const FactorialTable& table, long long n, long long k) {
    if (n < 0 || k < 0 || k > n) return 0;
    return mod_multiply(table.factorial(n), table.inverse_factorial(n - k), table.modulus());
}
// 順序を無視する組合せ。binomial と同じ値・条件。
template <class Table> long long combinations(const Table& table, long long n, long long k) {
    return binomial(table, n, k);
}
} // namespace cp
