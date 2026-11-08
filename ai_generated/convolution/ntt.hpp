#pragma once
#include <algorithm>
#include <array>
#include <cstddef>
#include <stdexcept>
#include <vector>
#include "detail/common.hpp"

namespace cp {
template <int Mod = 998244353, int Root = 3> constexpr std::size_t ntt_max_size() {
    return convolution_detail::NTTTraits<Mod, Root>::capacity;
}
// 自然順→自然順。順変換はΣ a[j] w^(jk)、逆変換はw^-1と1/n。
// 空は何もしない。非空は2冪長かつn|(Mod-1)。負の入力も正規化する。
template <int Mod = 998244353, int Root = 3> void ntt(std::vector<int>& a, bool inverse = false) {
    using Traits = convolution_detail::NTTTraits<Mod, Root>;
    const auto n = a.size();
    if (!n) return;
    if (!convolution_detail::power_of_two(n)) throw std::invalid_argument("NTT length must be a power of two");
    if (n > Traits::capacity) throw std::length_error("NTT length exceeds modulus capacity");
    static const auto roots = [] {
        std::array<std::array<int, 2>, Traits::rank + 1> r{};
        r[Traits::rank][0] = static_cast<int>(convolution_detail::power(Root, (Mod - 1) / Traits::capacity, Mod));
        r[Traits::rank][1] = static_cast<int>(convolution_detail::power(r[Traits::rank][0], Mod - 2, Mod));
        for (int k = Traits::rank; k > 0; --k) for (int d = 0; d < 2; ++d)
            r[k - 1][d] = static_cast<int>(static_cast<long long>(r[k][d]) * r[k][d] % Mod);
        return r;
    }();
    for (auto& x : a) { x %= Mod; if (x < 0) x += Mod; }
    for (std::size_t i = 1, j = 0; i < n; ++i) {
        auto bit = n >> 1;
        for (; j & bit; bit >>= 1) j ^= bit;
        j ^= bit;
        if (i < j) std::swap(a[i], a[j]);
    }
    int stage = 1;
    for (std::size_t len = 2; len <= n; len *= 2, ++stage) {
        const int root = roots[stage][inverse ? 1 : 0];
        for (std::size_t first = 0; first < n; first += len) {
            long long w = 1;
            for (std::size_t j = 0; j < len / 2; ++j) {
                const auto u = a[first + j], v = static_cast<int>(a[first + j + len / 2] * w % Mod);
                const long long sum = static_cast<long long>(u) + v;
                a[first + j] = static_cast<int>(sum >= Mod ? sum - Mod : sum);
                a[first + j + len / 2] = u >= v ? u - v : u - v + Mod;
                w = w * root % Mod;
            }
        }
        if (len == n) break;
    }
    if (inverse) {
        const auto scale = convolution_detail::power(static_cast<long long>(n), Mod - 2, Mod);
        for (auto& x : a) x = static_cast<int>(x * scale % Mod);
    }
}
} // namespace cp
