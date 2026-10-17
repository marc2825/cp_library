#pragma once
#include <cassert>
#include <cstdint>

namespace cp {
inline std::uint64_t gray_encode(std::uint64_t x) { return x^(x>>1); }
inline std::uint64_t gray_decode(std::uint64_t x) {
    for (int shift=1; shift<64; shift*=2) x^=x>>shift;
    return x;
}
// bits<=63。64bit単体の変換にはencode/decodeを使用。
template<class Callback> void for_each_gray_code(int bits, Callback callback) {
    assert(0<=bits && bits<=63);
    for (std::uint64_t i=0; i<(std::uint64_t(1)<<bits); ++i) callback(gray_encode(i));
}
} // namespace cp
