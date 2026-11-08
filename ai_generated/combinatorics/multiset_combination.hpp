#pragma once
#include <stdexcept>
#include "binomial.hpp"

namespace cp {
// n種類から重複を許してk個。H(n,k)=C(n+k-1,k)、H(0,0)=1。
template <class Table> long long multiset_combination(const Table& table, long long n, long long k) {
    if (n < 0 || k < 0) return 0;
    if (!k) return 1 % table.modulus();
    if (!n) return 0;
    // n+k-1 を作る前に上限判定し、long long の加算もあふれさせない。
    if (n - 1 > table.max_n() || k > table.max_n() - (n - 1))
        throw std::out_of_range("multiset combination requires table through n+k-1");
    return binomial(table, n + k - 1, k);
}
} // namespace cp
