#pragma once
#include <cassert>
#include <cstdint>
#include "../set_function/submask_enumeration.hpp"

namespace cp {
// n<=63。空集合を含む全(S,T), T⊆S。O(3^n)回、保存領域O(1)。
template<class Callback> void for_each_subset_pair(int n, Callback callback) {
    assert(0<=n && n<=63);
    for (std::uint64_t s=0;s<(std::uint64_t(1)<<n);++s)
        for_each_submask(s,[&](std::uint64_t t){ callback(s,t); });
}
} // namespace cp
