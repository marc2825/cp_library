#pragma once
#include "polynomial.hpp"
namespace cp {
// Return S with S(0)=0 and S(x+1)-S(x)=f(x). Degree(f)+1 < Mod.
template <int Mod = 998244353> std::vector<int> polynomial_prefix_sum(const std::vector<int> &input) {
    using F = fps_detail::Field<Mod>;
    auto a = polynomial_trim<Mod>(input);
    if (a.empty())
        return {};
    if (a.size() >= static_cast<std::size_t>(Mod))
        throw std::domain_error("prefix polynomial requires degree+1 < characteristic");
    const auto n = a.size();
    std::vector<std::vector<int>> choose(n + 1);
    choose[0] = {1};
    for (std::size_t i = 1; i <= n; ++i) {
        choose[i].resize(i + 1, 1);
        for (std::size_t j = 1; j < i; ++j)
            choose[i][j] = F::add(choose[i - 1][j - 1], choose[i - 1][j]);
    }
    auto inv = fps_detail::inverses<Mod>(n);
    std::vector<int> result(n + 1);
    for (std::size_t k = n; k > 0; --k) {
        result[k] = F::mul(a[k - 1], inv[k]);
        for (std::size_t j = 0; j < k; ++j)
            a[j] = F::sub(a[j], F::mul(result[k], choose[k][j]));
    }
    return result;
}
} // namespace cp
