#pragma once
#include <algorithm>
#include <cstddef>
#include <stdexcept>
#include <vector>
#include "../fps/detail/common.hpp"

namespace cp {
// NTT上限を超える結果も、ブロック対の通常積を足して求める。
// block_sizeは正の2冪かつNTT容量/2以下（容量1では1）。
template <int Mod = 998244353, int Root = 3> std::vector<int> convolution_large(
    const std::vector<int>& a, const std::vector<int>& b,
    std::size_t block_size = fps_max_size<Mod, Root>()) {
    using F = fps_detail::Field<Mod>;
    if (!convolution_detail::power_of_two(block_size) || block_size > fps_max_size<Mod, Root>())
        throw std::invalid_argument("invalid convolution block size");
    if (a.empty() || b.empty()) return {};
    if (a.size() > std::vector<int>().max_size() - (b.size() - 1)) throw std::length_error("convolution output is too large");
    std::vector<int> result(a.size() + b.size() - 1);
    for (std::size_t i = 0; i < a.size();) {
        const auto alen = std::min(block_size, a.size() - i);
        const std::vector<int> x(a.begin() + i, a.begin() + i + alen);
        for (std::size_t j = 0; j < b.size();) {
            const auto blen = std::min(block_size, b.size() - j);
            const std::vector<int> y(b.begin() + j, b.begin() + j + blen);
            const auto product = fps_detail::multiply<Mod, Root>(x, y);
            for (std::size_t k = 0; k < product.size(); ++k) result[i + j + k] = F::add(result[i + j + k], product[k]);
            j += blen;
        }
        i += alen;
    }
    return result;
}
} // namespace cp
