#pragma once
#include "polynomial.hpp"
namespace cp {
// f(x/(1-c*x)) mod x^n. Inverse is obtained by replacing c with -c. O(n^2), no division.
template <int Mod = 998244353>
std::vector<int> fps_fractional_linear_transform(const std::vector<int> &f, long long c, std::size_t n) {
    using F = fps_detail::Field<Mod>;
    std::vector<int> out(n);
    if (!n)
        return out;
    out[0] = f.empty() ? 0 : F::norm(f[0]);
    int shift = F::norm(c);
    std::vector<int> row(n);
    row[0] = 1;
    // [x^k](x/(1-cx))^j = binom(k-1,j-1)c^(k-j).
    for (std::size_t k = 1; k < n; ++k) {
        for (std::size_t j = 1; j <= k && j < f.size(); ++j)
            out[k] = F::add(out[k], F::mul(F::norm(f[j]), row[j - 1]));
        for (std::size_t j = k; j > 0; --j)
            row[j] = F::add(F::mul(shift, row[j]), row[j - 1]);
        row[0] = F::mul(shift, row[0]);
    }
    return out;
}
} // namespace cp
