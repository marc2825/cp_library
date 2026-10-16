#pragma once
#include <cstddef>
#include <type_traits>
#include <vector>
#include "ntt.hpp"

namespace cp {
namespace convolution_detail {
template <int Mod, class T> int normalize(T value) {
    static_assert(std::is_integral_v<T> && !std::is_same_v<T, bool> && sizeof(T) <= 8,
                  "convolution coefficients must be integral, at most 64 bits, and not bool");
    if constexpr (std::is_signed_v<T>) {
        const auto r = static_cast<long long>(value) % Mod;
        return static_cast<int>(r < 0 ? r + Mod : r);
    } else return static_cast<int>(static_cast<unsigned long long>(value) % Mod);
}
} // namespace convolution_detail
// c[k]=Σ_{i+j=k}a[i]b[j] mod Mod。空を含む場合は空。係数は昇冪順。
template <int Mod = 998244353, int Root = 3, class T, class U>
std::vector<int> convolution(const std::vector<T>& a, const std::vector<U>& b) {
    const auto sizes = convolution_detail::shape(a.size(), b.size(), ntt_max_size<Mod, Root>());
    if (!sizes.first) return {};
    std::vector<int> x(sizes.second), y(sizes.second);
    for (std::size_t i = 0; i < a.size(); ++i) x[i] = convolution_detail::normalize<Mod>(a[i]);
    for (std::size_t i = 0; i < b.size(); ++i) y[i] = convolution_detail::normalize<Mod>(b[i]);
    ntt<Mod, Root>(x); ntt<Mod, Root>(y);
    for (std::size_t i = 0; i < x.size(); ++i) x[i] = static_cast<int>(static_cast<long long>(x[i]) * y[i] % Mod);
    ntt<Mod, Root>(x, true);
    x.resize(sizes.first);
    return x;
}
} // namespace cp
