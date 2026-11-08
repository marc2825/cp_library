#pragma once
#include <stdexcept>
#include <vector>
#include "detail/arithmetic.hpp"
#include "../number_theory_basic/mod_pow.hpp"

namespace cp {
// Montmort数D(0)=1,D(1)=0、D(n)=(n-1)(D(n-1)+D(n-2))。
inline std::vector<long long> derangements(int max_n,long long modulus=998244353) {
    if (max_n<0 || modulus<=0) throw std::invalid_argument("invalid derangement bound or modulus");
    std::vector<long long> answer(static_cast<std::size_t>(max_n)+1); answer[0]=1%modulus;
    for (long long n=2;n<=max_n;++n)
        answer[n]=mod_multiply(n-1,combinatorics_detail::add_mod(answer[n-1],answer[n-2],modulus),modulus);
    return answer;
}
} // namespace cp
