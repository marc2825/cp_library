#pragma once
#include "../fps/detail/common.hpp"
namespace cp {
// h[k]=sum_i a[k+i]b[i]. A cyclic product of size ceil_pow2(|a|) suffices: wrap occurs below |b|-1.
template <int Mod = 998244353, int Root = 3>
std::vector<int> fast_middle_product(const std::vector<int> &a, const std::vector<int> &b) {
    using F = fps_detail::Field<Mod>;
    if (b.empty() || a.size() < b.size())
        throw std::invalid_argument("middle product needs 0 < |b| <= |a|");
    if (a.size() > ntt_max_size<Mod, Root>())
        throw std::length_error("middle product exceeds NTT capacity");
    auto count = a.size() - b.size() + 1;
    std::vector<int> result(count);
    if (std::min(count, b.size()) <= 16) {
        for (std::size_t i = 0; i < count; ++i)
            for (std::size_t j = 0; j < b.size(); ++j)
                result[i] = F::add(result[i], F::mul(F::norm(a[i + j]), F::norm(b[j])));
        return result;
    }
    std::size_t size = 1;
    while (size < a.size())
        size *= 2;
    auto x = fps_detail::prefix<Mod>(a, size);
    std::vector<int> y(size);
    for (std::size_t i = 0; i < b.size(); ++i)
        y[i] = F::norm(b[b.size() - 1 - i]);
    ntt<Mod, Root>(x);
    ntt<Mod, Root>(y);
    for (std::size_t i = 0; i < size; ++i)
        x[i] = F::mul(x[i], y[i]);
    ntt<Mod, Root>(x, true);
    std::copy_n(x.begin() + b.size() - 1, count, result.begin());
    return result;
}
} // namespace cp
