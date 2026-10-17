#pragma once
#include <array>
#include <cstdint>

namespace cp {
namespace nim_product_detail {
inline const std::array<std::array<std::uint8_t, 256>, 256>& byte_table() {
    static const auto table = [] {
        std::array<std::array<std::uint8_t, 16>, 16> small{};
        // 小さい体はmexの定義から独立に構築する。
        for (unsigned a = 0; a < 16; ++a) for (unsigned b = 0; b < 16; ++b) {
            std::array<bool, 16> seen{};
            for (unsigned x = 0; x < a; ++x) for (unsigned y = 0; y < b; ++y)
                seen[small[x][b] ^ small[a][y] ^ small[x][y]] = true;
            unsigned value = 0; while (value < 16 && seen[value]) ++value;
            small[a][b] = static_cast<std::uint8_t>(value);
        }
        std::array<std::array<std::uint8_t, 256>, 256> result{};
        for (unsigned a = 0; a < 256; ++a) for (unsigned b = 0; b < 256; ++b) {
            const auto low = small[a & 15][b & 15], high = small[a >> 4][b >> 4];
            const auto upper = small[(a & 15) ^ (a >> 4)][(b & 15) ^ (b >> 4)] ^ low;
            result[a][b] = static_cast<std::uint8_t>(low ^ small[high][8] ^ (upper << 4));
        }
        return result;
    }();
    return table;
}
inline std::uint64_t multiply(std::uint64_t a, std::uint64_t b, unsigned bits) {
    if (!a || !b) return 0;
    if (a < 256 && b < 256) return byte_table()[a][b];
    const unsigned half = bits / 2;
    const auto mask = (std::uint64_t(1) << half) - 1;
    const auto a0 = a & mask, a1 = a >> half, b0 = b & mask, b1 = b >> half;
    const auto low = multiply(a0, b0, half), high = multiply(a1, b1, half);
    const auto upper = multiply(a0 ^ a1, b0 ^ b1, half) ^ low;
    return low ^ multiply(high, std::uint64_t(1) << (half - 1), half) ^ (upper << half);
}
} // namespace nim_product_detail
// uint64全域のnim積。加算はXOR、通常の整数積ではない。
// docs/game_theory/nim_product.md
inline std::uint64_t nim_product(std::uint64_t a, std::uint64_t b) {
    return nim_product_detail::multiply(a, b, 64);
}
} // namespace cp
