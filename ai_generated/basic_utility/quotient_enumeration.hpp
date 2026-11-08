#pragma once
#include <cstdint>

namespace cp {
// floor(n/i) が等しい最大の閉区間 [l,r] と商を渡す。n=0は呼出しなし。
// UINT64_MAXでもr+1を計算しない。O(sqrt(n))回のコールバック。
template<class Callback> void for_each_quotient(std::uint64_t n, Callback callback) {
    for (std::uint64_t l=1; l<=n;) {
        auto q=n/l, r=n/q;
        callback(l,r,q);
        if (r==n) break;
        l=r+1;
    }
}
} // namespace cp
