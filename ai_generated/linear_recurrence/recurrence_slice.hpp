#pragma once
#include "../fps/inv.hpp"
#include "kitamasa.hpp"
#include <limits>
namespace cp {
// a[start..start+count), using one Kitamasa shift followed by a rational-series expansion.
// O(d^2 log(start+1) + d^2 + M(d+count)), no assumption on nonzero trailing recurrence coefficient.
template <int Mod = 998244353, int Root = 3>
std::vector<int> linear_recurrence_slice(const std::vector<int> &initial,
                                         const std::vector<int> &coefficients, std::uint64_t start,
                                         std::size_t count) {
    using F = fps_detail::Field<Mod>;
    linear_recurrence_detail::validate(initial, coefficients);
    if (count && count - 1 > std::numeric_limits<std::uint64_t>::max() - start)
        throw std::length_error("recurrence index overflow");
    fps_detail::check_precision<Mod, Root>(std::max(count, initial.size() + 1));
    if (!count)
        return {};
    auto c = linear_recurrence_detail::normalize<Mod>(coefficients);
    auto weights = kitamasa_coefficients<Mod>(c, start);
    const auto d = c.size();
    std::vector<int> prefix(std::min(d, count));
    for (std::size_t k = 0; k < prefix.size(); ++k) {
        for (std::size_t i = 0; i < d; ++i)
            prefix[k] = F::add(prefix[k], F::mul(weights[i], F::norm(initial[i])));
        int high = weights.back();
        for (std::size_t i = d - 1; i > 0; --i)
            weights[i] = F::add(weights[i - 1], F::mul(high, c[d - 1 - i]));
        weights[0] = F::mul(high, c[d - 1]);
    }
    if (count <= d)
        return prefix;
    std::vector<int> q(d + 1);
    q[0] = 1;
    for (std::size_t i = 0; i < d; ++i)
        q[i + 1] = F::sub(0, c[i]);
    auto p = fps_detail::product_prefix<Mod, Root>(prefix, q, d);
    return fps_detail::product_prefix<Mod, Root>(p, fps_inv<Mod, Root>(q, count), count);
}
} // namespace cp
