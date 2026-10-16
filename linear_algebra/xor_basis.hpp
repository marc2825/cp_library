#pragma once
#include <algorithm>
#include <array>
#include <cstdint>
#include <limits>
#include <type_traits>
#include <vector>

namespace cp {
// GF(2)の基底。符号なし整数をビットベクトルとして扱い、数値順でmin/maxを求める。
template <class UInt = std::uint64_t> class XorBasis {
    static_assert(std::is_integral_v<UInt> && std::is_unsigned_v<UInt> && !std::is_same_v<UInt, bool>, "XorBasis requires an unsigned integer type");
    static constexpr int bits = std::numeric_limits<UInt>::digits;
    std::array<UInt, bits> pivots_{};
    int rank_ = 0;
public:
    int rank() const { return rank_; }
    bool empty() const { return !rank_; }
    void clear() { pivots_.fill(UInt(0)); rank_ = 0; }
    // 独立ならtrue。0・既存の張る空間の要素はfalseで基底不変。
    bool insert(UInt value) {
        for (int bit = bits; bit-- > 0;) if ((value >> bit) & UInt(1)) {
            if (pivots_[bit]) value ^= pivots_[bit];
            else { pivots_[bit] = value; ++rank_; return true; }
        }
        return false;
    }
    UInt min_xor(UInt seed = 0) const {
        for (int bit = bits; bit-- > 0;) seed = std::min(seed, UInt(seed ^ pivots_[bit]));
        return seed;
    }
    UInt max_xor(UInt seed = 0) const {
        for (int bit = bits; bit-- > 0;) seed = std::max(seed, UInt(seed ^ pivots_[bit]));
        return seed;
    }
    bool contains(UInt value) const { return min_xor(value) == 0; }
    // 空間の和を取り、増えた次元を返す。自分自身とのmergeも可。
    int merge(const XorBasis& other) { const int before = rank_; for (const auto& x : other.pivots_) insert(x); return rank_ - before; }
    // pivotの最上位ビットの降順。挿入順で値は変わり得る（標準形ではない）。
    std::vector<UInt> basis() const { std::vector<UInt> values; for (int bit = bits; bit-- > 0;) if (pivots_[bit]) values.push_back(pivots_[bit]); return values; }
};
} // namespace cp
