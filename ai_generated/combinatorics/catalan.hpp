#pragma once
#include <stdexcept>
#include "binomial.hpp"

namespace cp {
// C_n = binom(2n,n)-binom(2n,n+1)。除算せず、Pascal表なら合成数法でも可。
template <class Table> long long catalan(const Table& table, long long n) {
    if (n < 0) return 0;
    if (n > table.max_n() / 2) throw std::out_of_range("Catalan query requires table through 2*n");
    return combinatorics_detail::subtract_mod(binomial(table, 2 * n, n), binomial(table, 2 * n, n + 1), table.modulus());
}
} // namespace cp
