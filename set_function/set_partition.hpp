#pragma once
#include <cstddef>
#include <vector>
#include "set_power_series.hpp"

namespace cp {
// 非空ブロックBの重みweights[B]。weights[0]は無視。空集合の分割は1通り。
template <int Mod = 998244353> std::vector<int> set_partition_sum(const std::vector<int>& weights) {
    set_function_detail::validate_size(weights.size()); auto f = weights; f[0] = 0; return sps_exp<Mod>(f);
}
// 戻り値[ブロック数][集合]。最小要素を含むブロックを選んで重複を除く。
// O(n 3^n)時間・O(n 2^n)領域。全体和だけならset_partition_sumを使う。
template <int Mod = 998244353> std::vector<std::vector<int>> set_partition_by_blocks(const std::vector<int>& weights) {
    using F = linear_recurrence_detail::Arithmetic<Mod>;
    set_function_detail::validate_size(weights.size());
    std::size_t bits = 0; for (auto n = weights.size(); n > 1; n >>= 1) ++bits;
    std::vector<std::vector<int>> result(bits + 1, std::vector<int>(weights.size())); result[0][0] = 1;
    for (std::size_t mask = 1; mask < weights.size(); ++mask) {
        const auto bit = mask & (~mask + 1), rest = mask ^ bit;
        for (auto sub = rest;; sub = (sub - 1) & rest) {
            const auto block = sub | bit;
            for (std::size_t k = 1; k <= bits; ++k) result[k][mask] = F::add(result[k][mask], F::mul(F::norm(weights[block]), result[k - 1][mask ^ block]));
            if (!sub) break;
        }
    }
    return result;
}
} // namespace cp
