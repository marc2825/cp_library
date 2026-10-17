#pragma once
#include <stdexcept>
#include <vector>
#include "detail/arithmetic.hpp"

namespace cp {
// Bell三角形。O(N^2)加算、O(N)領域。合成数法・法1も可。
inline std::vector<long long> bell_numbers(int max_n,long long modulus=998244353) {
    if (max_n<0 || modulus<=0) throw std::invalid_argument("invalid Bell bound or modulus");
    std::vector<long long> answer{1%modulus},row{1%modulus};
    for (long long n=1;n<=max_n;++n) {
        std::vector<long long> next(static_cast<std::size_t>(n)+1); next[0]=row.back();
        for (long long k=1;k<=n;++k) next[k]=combinatorics_detail::add_mod(next[k-1],row[k-1],modulus);
        answer.push_back(next[0]); row.swap(next);
    }
    return answer;
}
} // namespace cp
