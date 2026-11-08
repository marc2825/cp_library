#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
#include <string_view>
#include <utility>
#include <vector>

namespace cp {
// バイト列のLCS長。64bit間のshift carryと減算borrowを伝播する。
// O(|a| ceil(|b|/64))時間、O(256 ceil(|b|/64))領域。短い側をbit列にする。
inline std::size_t bit_parallel_lcs_length(std::string_view a, std::string_view b) {
    if (a.size() < b.size()) std::swap(a, b);
    if (b.empty()) return 0;
    const std::size_t words = (b.size() - 1) / 64 + 1;
    std::array<std::vector<std::uint64_t>, 256> masks;
    for (auto& mask : masks) mask.resize(words);
    for (std::size_t i = 0; i < b.size(); ++i)
        masks[static_cast<unsigned char>(b[i])][i / 64] |= std::uint64_t(1) << (i % 64);
    std::vector<std::uint64_t> row(words);
    for (unsigned char ch : a) {
        std::uint64_t carry = 1, borrow = 0;
        for (std::size_t j = 0; j < words; ++j) {
            const auto old = row[j], x = old | masks[ch][j], y = (old << 1) | carry;
            carry = old >> 63;
            const auto first = x - y, difference = first - borrow;
            borrow = (x < y) | (first < borrow);
            row[j] = x & ~difference;
        }
    }
    std::size_t length = 0;
    for (auto word : row) length += static_cast<std::size_t>(__builtin_popcountll(word));
    return length;
}
} // namespace cp
