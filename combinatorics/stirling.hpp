#pragma once
#include <stdexcept>
#include <vector>
#include "detail/arithmetic.hpp"
#include "../number_theory_basic/mod_pow.hpp"

namespace cp {
// 0<=n<=max_nの三角表。第1種は既定で符号なし、signed_values=trueで符号付き。
inline std::vector<std::vector<long long>> stirling_first(int max_n,long long modulus=998244353,bool signed_values=false) {
    if (max_n<0 || modulus<=0) throw std::invalid_argument("invalid Stirling bound or modulus");
    std::vector<std::vector<long long>> table{{1%modulus}};
    for (long long n=1;n<=max_n;++n) {
        table.emplace_back(static_cast<std::size_t>(n)+1);
        for (long long k=1;k<=n;++k) {
            auto a=table[n-1][k-1],b=k<n?mod_multiply(n-1,table[n-1][k],modulus):0;
            table[n][k]=signed_values?combinatorics_detail::subtract_mod(a,b,modulus):combinatorics_detail::add_mod(a,b,modulus);
        }
    }
    return table;
}
inline std::vector<std::vector<long long>> stirling_second(int max_n,long long modulus=998244353) {
    if (max_n<0 || modulus<=0) throw std::invalid_argument("invalid Stirling bound or modulus");
    std::vector<std::vector<long long>> table{{1%modulus}};
    for (long long n=1;n<=max_n;++n) {
        table.emplace_back(static_cast<std::size_t>(n)+1);
        for (long long k=1;k<=n;++k)
            table[n][k]=combinatorics_detail::add_mod(table[n-1][k-1],k<n?mod_multiply(k,table[n-1][k],modulus):0,modulus);
    }
    return table;
}
} // namespace cp
