#pragma once
#include <cstddef>
#include <cstdint>
#include <vector>
#include "../fps/bostan_mori.hpp"

namespace cp {
// sum_{k=0}^{count-1} polynomial(k)*ratio^k。countはuint64全域。
// (1-r*x)^(d+1)で重み列を消去し、さらに(1-x)でprefix和にする。
template <int Mod = 998244353, int Root = 3> int polynomial_geometric_sum(
    const std::vector<int>& polynomial, int ratio, std::uint64_t count) {
    using F = fps_detail::Field<Mod>;
    if (!count || polynomial.empty()) return 0;
    const auto p = polynomial_trim<Mod>(polynomial);
    if (p.empty()) return 0;
    if (p.size() >= fps_max_size<Mod, Root>() - 1) throw std::length_error("weighted geometric degree exceeds FPS capacity");
    ratio = F::norm(ratio);
    std::vector<int> q{1}, sequence(p.size()); int weight = 1;
    for (std::size_t i = 0; i < p.size(); ++i) {
        sequence[i] = F::mul(polynomial_evaluate<Mod>(p, static_cast<int>(i % Mod)), weight); weight = F::mul(weight, ratio);
        q = polynomial_multiply<Mod, Root>(q, {1, F::sub(0, ratio)});
    }
    const auto numerator = fps_detail::product_prefix<Mod, Root>(sequence, q, sequence.size());
    q = polynomial_multiply<Mod, Root>(q, {1, F::sub(0, 1)});
    return bostan_mori<Mod, Root>(numerator, q, count - 1);
}
} // namespace cp
