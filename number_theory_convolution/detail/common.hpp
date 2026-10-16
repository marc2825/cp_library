#pragma once
#include <algorithm>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>
#include "arithmetic.hpp"
#include "../../number_theory_basic/prime_sieve.hpp"

namespace cp { namespace detail {
inline int arithmetic_bound(std::size_t size) {
    if (size > static_cast<std::size_t>(std::numeric_limits<int>::max()) + 1)
        throw std::length_error("arithmetic function bound exceeds int");
    return size ? static_cast<int>(size - 1) : 0;
}
// 逆変換は全更新の逆順。添字0には触れない。
template <bool Divisor, bool Inverse, class T>
void arithmetic_transform(std::vector<T>& f, const std::vector<int>& primes) {
    const int n = arithmetic_bound(f.size());
    const auto apply = [&](int p) {
        const auto update = [&](int k) {
            const int target = Divisor ? k * p : k, source = Divisor ? k : k * p;
            if constexpr (Inverse) arithmetic_subtract_assign(f[target], f[source]);
            else arithmetic_add_assign(f[target], f[source]);
        };
        if constexpr (Divisor != Inverse) for (int k = 1; k <= n / p; ++k) update(k);
        else for (int k = n / p; k >= 1; --k) update(k);
    };
    if constexpr (Inverse) for (auto it = primes.rbegin(); it != primes.rend(); ++it) apply(*it);
    else for (int p : primes) apply(p);
}
template <class T>
std::pair<std::vector<T>, std::vector<T>> arithmetic_inputs(const std::vector<T>& a, const std::vector<T>& b) {
    const auto size = std::max(a.size(), b.size());
    arithmetic_bound(size);
    std::vector<T> f(size, T(0)), g(size, T(0));
    for (std::size_t i = 1; i < a.size(); ++i) f[i] = a[i];
    for (std::size_t i = 1; i < b.size(); ++i) g[i] = b[i];
    return {std::move(f), std::move(g)};
}
} } // namespace cp::detail
