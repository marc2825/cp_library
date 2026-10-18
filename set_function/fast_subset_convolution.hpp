#pragma once
#include <vector>
#include "subset_convolution.hpp"
#include "../linear_algebra/detail/prime_field.hpp"

namespace cp {
// 既存のranked-zeta O(n^2 2^n)実装を素数modのint配列から使う入口。
template <int Mod = 998244353> std::vector<int> subset_convolution_mod(const std::vector<int>& a, const std::vector<int>& b) {
    using F = linear_algebra_detail::PrimeField<Mod>;
    std::vector<F> x(a.begin(), a.end()), y(b.begin(), b.end());
    const auto product = subset_convolution(x, y); std::vector<int> result;
    result.reserve(product.size()); for (auto value : product) result.push_back(value.value);
    return result;
}
} // namespace cp
