#pragma once
#include <cassert>
#include <cstdint>

namespace cp {
// [l,r) 内の非負倍数を昇順に列挙。step>0。0も倍数。
template<class Callback> void for_each_multiple(std::uint64_t l, std::uint64_t r,
                                                std::uint64_t step, Callback callback) {
    assert(l<=r && step>0);
    if (l==r) return;
    auto rem=l%step;
    if (rem) {
        auto delta=step-rem;
        if (delta>=r-l) return;
        l+=delta;
    }
    while (l<r) {
        callback(l);
        if (step>=r-l) break;
        l+=step;
    }
}
} // namespace cp
