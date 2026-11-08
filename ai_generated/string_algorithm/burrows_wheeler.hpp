#pragma once
#include "suffix_array.hpp"
#include <array>
#include <cstdint>
#include <string>

namespace cp {
// Virtual terminator 0, bytes 0..255 encoded as 1..256. Exactly n+1 symbols, including empty text.
struct BurrowsWheelerTransform {
    std::vector<std::uint16_t> last;
    int primary = 0;
};
inline BurrowsWheelerTransform burrows_wheeler_transform(std::string_view text) {
    if (text.size() >= static_cast<std::size_t>(std::numeric_limits<int>::max()))
        throw std::length_error("BWT text too long");
    std::vector<int> s;
    for (unsigned char c : text)
        s.push_back(int(c) + 1);
    s.push_back(0);
    const auto sa = suffix_array(s);
    BurrowsWheelerTransform b;
    b.last.resize(s.size());
    for (std::size_t i = 0; i < sa.size(); ++i) {
        b.last[i] = static_cast<std::uint16_t>(s[sa[i] ? sa[i] - 1 : s.size() - 1]);
        if (!sa[i])
            b.primary = static_cast<int>(i);
    }
    return b;
}
inline std::string inverse_burrows_wheeler(const BurrowsWheelerTransform &b) {
    const int n = string_algorithm_detail::checked_size(b.last.size());
    if (!n || b.primary < 0 || b.primary >= n)
        throw std::invalid_argument("invalid BWT primary index");
    std::array<int, 257> count{}, start{};
    std::vector<int> rank(n);
    for (int i = 0; i < n; ++i) {
        if (b.last[i] > 256)
            throw std::invalid_argument("invalid BWT symbol");
        rank[i] = count[b.last[i]]++;
    }
    if (count[0] != 1 || b.last[b.primary] != 0)
        throw std::invalid_argument("invalid BWT terminator");
    for (int c = 1; c <= 256; ++c)
        start[c] = start[c - 1] + count[c - 1];
    std::string out(n - 1, '\0');
    int row = b.primary;
    for (int i = n - 1; i-- > 0;) {
        row = start[b.last[row]] + rank[row];
        if (!b.last[row])
            throw std::invalid_argument("BWT LF mapping has a short cycle");
        out[i] = static_cast<char>(b.last[row] - 1);
    }
    row = start[b.last[row]] + rank[row];
    if (row != b.primary)
        throw std::invalid_argument("invalid BWT LF cycle");
    return out;
}
} // namespace cp
