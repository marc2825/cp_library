#pragma once
#include <cstddef>
#include <stdexcept>
#include <vector>
#include "../fps/detail/common.hpp"

namespace cp {
// h[k]=sum_i a[k+i]*b[i], k=0..|a|-|b|。bは非空、|a|>=|b|。
// 反転したbとの全長積から中央を抽出する。専用NTT最適化はしない。
template <int Mod = 998244353, int Root = 3>
std::vector<int> middle_product(const std::vector<int>& a, const std::vector<int>& b) {
    if (b.empty() || a.size() < b.size()) throw std::invalid_argument("middle product needs 0 < |b| <= |a|");
    const auto product = fps_detail::multiply<Mod, Root>(a, std::vector<int>(b.rbegin(), b.rend()));
    return {product.begin() + b.size() - 1, product.begin() + a.size()};
}
} // namespace cp
