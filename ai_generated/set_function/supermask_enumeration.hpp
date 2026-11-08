#pragma once
#include <cstdint>
#include <stdexcept>

namespace cp {
// 下位bitsビットの全体集合内でmaskの上位集合を昇順列挙。mask自身と全体を含む。
// bitsは0..64。maskが全体集合に収まらなければcallback前にinvalid_argument。
template <class Callback>
void for_each_supermask(std::uint64_t mask, int bits, Callback callback) {
    if (bits < 0 || bits > 64) throw std::invalid_argument("supermask bit count must be in [0,64]");
    const auto full = bits == 64 ? ~std::uint64_t(0) : (std::uint64_t(1) << bits) - 1;
    if (mask & ~full) throw std::invalid_argument("mask lies outside the specified universe");
    for (auto super = mask;; super = (super + 1) | mask) {
        callback(super);
        if (super == full) break;
    }
}
} // namespace cp
