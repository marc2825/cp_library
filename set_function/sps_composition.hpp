#pragma once
#include <algorithm>
#include <cstddef>
#include <vector>
#include "fast_subset_convolution.hpp"

namespace cp {
// 有限多項式 outer(inner) をSPS積で計算。inner[0]!=0も可。
// 外側の全係数を使用。n変数なら非定数部分の(n+1)乗は0。
template <int Mod = 998244353> std::vector<int> sps_compose(const std::vector<int>& outer, const std::vector<int>& inner) {
    using F = linear_recurrence_detail::Arithmetic<Mod>;
    set_function_detail::validate_size(inner.size());
    std::size_t bits = 0; for (auto n = inner.size(); n > 1; n >>= 1) ++bits;
    const int constant = F::norm(inner[0]);
    std::vector<int> shifted(bits + 1);
    // outer(constant+y) mod y^(bits+1)。除算なし、小標数でも可。
    for (std::size_t i = outer.size(); i-- > 0;) {
        for (std::size_t j = bits; j > 0; --j) shifted[j] = F::add(F::mul(shifted[j], constant), shifted[j - 1]);
        shifted[0] = F::add(F::mul(shifted[0], constant), F::norm(outer[i]));
    }
    auto nilpotent = inner; nilpotent[0] = 0;
    std::vector<int> result(inner.size());
    for (std::size_t j = std::min(bits + 1, outer.size()); j-- > 0;) {
        result = subset_convolution_mod<Mod>(result, nilpotent); result[0] = F::add(result[0], shifted[j]);
    }
    return result;
}
} // namespace cp
