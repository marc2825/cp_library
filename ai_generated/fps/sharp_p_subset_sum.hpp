#pragma once
#include "exp.hpp"
namespace cp {
// Counts subsets by total weight through degree limit. Weight 0 multiplies every answer by 2.
template <int Mod = 998244353, int Root = 3>
std::vector<int> sharp_p_subset_sum(const std::vector<std::size_t> &weights, std::size_t limit) {
    using F = fps_detail::Field<Mod>;
    if (limit >= static_cast<std::size_t>(Mod) || limit >= fps_max_size<Mod, Root>())
        throw std::length_error("subset-sum precision exceeds field/NTT capacity");
    std::vector<int> freq(limit + 1), log(limit + 1);
    int scale = 1;
    for (auto w : weights) {
        if (!w)
            scale = F::add(scale, scale);
        else if (w <= limit)
            freq[w] = F::add(freq[w], 1);
    }
    auto inv = fps_detail::inverses<Mod>(limit);
    for (std::size_t s = 1; s <= limit; ++s)
        if (freq[s])
            for (std::size_t k = 1; k <= limit / s; ++k) {
                int v = F::mul(freq[s], inv[k]);
                log[s * k] = k & 1 ? F::add(log[s * k], v) : F::sub(log[s * k], v);
            }
    auto out = fps_exp<Mod, Root>(log, limit + 1);
    for (auto &x : out)
        x = F::mul(x, scale);
    return out;
}
} // namespace cp
