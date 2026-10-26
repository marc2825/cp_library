#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
#include <string_view>
#include <utility>
#include <vector>

namespace cp {
// バイト列全体のLevenshtein距離。挿入・削除・置換の単位費用、転置なし。
// Myersの64bit block間に水平差分を渡す。長さ64の制限なし。
inline std::size_t myers_edit_distance(std::string_view a, std::string_view b) {
    if (a.size() < b.size()) std::swap(a, b);
    if (b.empty()) return a.size();
    const std::size_t words = (b.size() - 1) / 64 + 1;
    std::array<std::vector<std::uint64_t>, 256> masks;
    for (auto& mask : masks) mask.resize(words);
    for (std::size_t i = 0; i < b.size(); ++i)
        masks[static_cast<unsigned char>(b[i])][i / 64] |= std::uint64_t(1) << (i % 64);
    std::vector<std::uint64_t> positive(words, ~std::uint64_t(0)), negative(words);
    std::size_t distance = b.size();
    for (unsigned char ch : a) {
        int incoming = 1; // D(0,j)-D(0,j-1)=1: global alignment
        for (std::size_t j = 0; j < words; ++j) {
            auto eq = masks[ch][j];
            const auto pv = positive[j], mv = negative[j];
            const auto vertical = eq | mv;
            if (incoming < 0) eq |= 1;
            const auto horizontal = (((eq & pv) + pv) ^ pv) | eq;
            auto ph = mv | ~(horizontal | pv), mh = pv & horizontal;
            const unsigned last = j + 1 == words ? static_cast<unsigned>((b.size() - 1) % 64) : 63;
            const int outgoing = static_cast<int>((ph >> last) & 1) - static_cast<int>((mh >> last) & 1);
            ph = (ph << 1) | static_cast<std::uint64_t>(incoming > 0);
            mh = (mh << 1) | static_cast<std::uint64_t>(incoming < 0);
            positive[j] = mh | ~(vertical | ph);
            negative[j] = ph & vertical;
            incoming = outgoing;
        }
        if (incoming > 0) ++distance;
        if (incoming < 0) --distance;
    }
    return distance;
}
} // namespace cp
