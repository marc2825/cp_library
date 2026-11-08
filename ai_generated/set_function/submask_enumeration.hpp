#pragma once
#include <cstdint>

namespace cp {
// maskの部分集合を降順で列挙。既定で0も含む。uint64_tの最上位ビットも可。
template <class Callback>
void for_each_submask(std::uint64_t mask, Callback callback, bool include_empty = true) {
    for (auto sub = mask;; sub = (sub - 1) & mask) {
        if (sub || include_empty) callback(sub);
        if (!sub) break;
    }
}
} // namespace cp
