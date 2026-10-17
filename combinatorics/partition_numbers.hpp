#pragma once
#include <stdexcept>
#include <vector>
#include "detail/arithmetic.hpp"

namespace cp {
// 非負整数の分割数p(0)..p(N)。Eulerの五角数定理、O(N sqrt N)加減算/O(N)領域。
inline std::vector<long long> partition_numbers(int max_n,long long modulus=998244353) {
    if (max_n<0 || modulus<=0) throw std::invalid_argument("invalid partition bound or modulus");
    std::vector<long long> answer(static_cast<std::size_t>(max_n)+1); answer[0]=1%modulus;
    for (long long n=1;n<=max_n;++n) for (long long k=1;k*(3*k-1)/2<=n;++k) {
        for (auto p:{k*(3*k-1)/2,k*(3*k+1)/2}) if(p<=n)
            answer[n]=k%2?combinatorics_detail::add_mod(answer[n],answer[n-p],modulus):combinatorics_detail::subtract_mod(answer[n],answer[n-p],modulus);
    }
    return answer;
}
} // namespace cp
