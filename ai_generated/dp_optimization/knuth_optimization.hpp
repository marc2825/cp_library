#pragma once
#include <algorithm>
#include <stdexcept>
#include "interval_dp.hpp"

namespace cp {
// cost(l,r)は分割点に依存しない。opt[l][r-1]<=opt[l][r]<=opt[l+1][r]が必要。
// O(n^2)時間/領域。結果・半開区間・merge_order()はinterval_dpと共通。
template<class Cost> IntervalDPResult knuth_optimization(int n,Cost cost) {
    if (n<0) throw std::invalid_argument("negative interval dimension");
    IntervalDPResult result;
    result.dp.assign(static_cast<std::size_t>(n)+1,std::vector<long long>(static_cast<std::size_t>(n)+1));
    result.split.assign(static_cast<std::size_t>(n)+1,std::vector<int>(static_cast<std::size_t>(n)+1,-1));
    for (int length=2;length<=n;++length) for (int l=0;l<=n-length;++l) {
        int r=l+length;
        int lo=length==2?l+1:std::max(l+1,result.split[l][r-1]);
        int hi=length==2?r-1:std::min(r-1,result.split[l+1][r]);
        const long long c=cost(l,r);
        for (int k=lo;k<=hi;++k) {
            auto value=detail::dp_add(detail::dp_add(result.dp[l][k],result.dp[k][r]),c);
            if (result.split[l][r]<0 || value<result.dp[l][r]) { result.dp[l][r]=value; result.split[l][r]=k; }
        }
    }
    return result;
}
} // namespace cp
