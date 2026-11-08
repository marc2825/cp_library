#pragma once
#include <algorithm>
#include <cstddef>
#include <utility>
#include <vector>
#include "inv.hpp"
#include "polynomial.hpp"

namespace cp {
// a=b*q+r, deg(r)<deg(b)。{商,剰余}の末尾0を除き、零多項式は空。
template <int Mod = 998244353, int Root = 3> std::pair<std::vector<int>, std::vector<int>> polynomial_divmod(const std::vector<int>& a, const std::vector<int>& b) {
    using F = fps_detail::Field<Mod>;
    auto r = polynomial_trim<Mod>(a), divisor = polynomial_trim<Mod>(b);
    if (divisor.empty()) throw std::domain_error("polynomial divisor must be nonzero");
    fps_detail::check_precision<Mod, Root>(std::max(r.size(), divisor.size()));
    if (r.size() < divisor.size()) return {{}, r};
    const auto length = r.size() - divisor.size() + 1;
    std::vector<int> q(length);
    if (std::min(length, divisor.size()) <= 32) {
        const auto inverse = F::inv(divisor.back());
        for (std::size_t i = length; i-- > 0;) {
            q[i] = F::mul(r[i + divisor.size() - 1], inverse);
            for (std::size_t j = 0; j < divisor.size(); ++j) r[i + j] = F::sub(r[i + j], F::mul(q[i], divisor[j]));
        }
    } else {
        std::vector<int> ra(r.rbegin(), r.rend()), rb(divisor.rbegin(), divisor.rend());
        q = fps_detail::product_prefix<Mod, Root>(fps_detail::prefix<Mod>(ra, length), fps_inv<Mod, Root>(rb, length), length);
        std::reverse(q.begin(), q.end());
        r = polynomial_subtract<Mod>(r, fps_detail::multiply<Mod, Root>(divisor, q));
    }
    r.resize(divisor.size() - 1); fps_detail::trim(r); fps_detail::trim(q);
    return {q, r};
}
template <int Mod = 998244353, int Root = 3> std::vector<int> polynomial_quotient(const std::vector<int>& a, const std::vector<int>& b) {
    return polynomial_divmod<Mod, Root>(a, b).first;
}
template <int Mod = 998244353, int Root = 3> std::vector<int> polynomial_remainder(const std::vector<int>& a, const std::vector<int>& b) {
    return polynomial_divmod<Mod, Root>(a, b).second;
}
} // namespace cp
