#pragma once
#include "detail/finite_field_polynomial.hpp"
namespace cp {
// Zero input -> 0; two nonzero constants -> 1.
template <int Mod = 998244353> int polynomial_resultant(std::vector<int> a, std::vector<int> b) {
    using A = finite_polynomial_detail::Algebra<Mod>;
    using F = typename A::F;
    a = A::trim(std::move(a));
    b = A::trim(std::move(b));
    int result = 1;
    if (a.empty() || b.empty())
        return 0;
    while (b.size() > 1) {
        auto n = a.size() - 1, m = b.size() - 1;
        if (n < m) {
            if ((n & 1) && (m & 1))
                result = F::sub(0, result);
            std::swap(a, b);
            continue;
        }
        auto r = A::mod(a, b);
        if (r.empty())
            return 0;
        result = F::mul(result, F::power(b.back(), n - (r.size() - 1)));
        if ((n & 1) && (m & 1))
            result = F::sub(0, result);
        a = std::move(b);
        b = std::move(r);
    }
    return F::mul(result, F::power(b[0], a.size() - 1));
}
// Zero -> 0, nonzero constant/linear -> 1; valid in characteristic 2 as well.
template <int Mod = 998244353> int polynomial_discriminant(const std::vector<int> &polynomial) {
    using A = finite_polynomial_detail::Algebra<Mod>;
    using F = typename A::F;
    auto a = A::trim(polynomial);
    if (a.empty())
        return 0;
    if (a.size() <= 2)
        return 1;
    auto n = a.size() - 1;
    int r = F::mul(polynomial_resultant<Mod>(a, polynomial_derivative<Mod>(a)), F::inv(a.back()));
    return (n % 4 == 2 || n % 4 == 3) ? F::sub(0, r) : r;
}
} // namespace cp
