#pragma once
#include "ntt.hpp"
namespace cp {
// Natural-order transform of the zero-padded coefficients, from their length-n transform.
template <int Mod = 998244353, int Root = 3> void ntt_doubling(std::vector<int> &transformed) {
    const auto n = transformed.size();
    if (!n)
        return;
    if (!convolution_detail::power_of_two(n))
        throw std::invalid_argument("NTT doubling needs power-of-two length");
    if (n > ntt_max_size<Mod, Root>() / 2)
        throw std::length_error("doubled NTT exceeds capacity");
    auto odd = transformed;
    ntt<Mod, Root>(odd, true);
    const auto root = convolution_detail::power(Root, (Mod - 1) / (2 * n), Mod);
    long long w = 1;
    for (auto &x : odd) {
        x = static_cast<int>(x * w % Mod);
        w = w * root % Mod;
    }
    ntt<Mod, Root>(odd);
    std::vector<int> result(2 * n);
    for (std::size_t i = 0; i < n; ++i) {
        int x = transformed[i] % Mod;
        if (x < 0)
            x += Mod;
        result[2 * i] = x;
        result[2 * i + 1] = odd[i];
    }
    transformed.swap(result);
}
} // namespace cp
