#pragma once
#include "detail/common.hpp"

namespace cp {
// c[n]=sum_{i*j=n}a[i]*b[j]。長さmax(a.size(),b.size())で打ち切る。
template <class T>
std::vector<T> dirichlet_convolution(const std::vector<T>& a, const std::vector<T>& b) {
    const auto size = std::max(a.size(), b.size());
    const auto n = static_cast<std::size_t>(detail::arithmetic_bound(size));
    std::vector<T> result(size, T(0));
    for (std::size_t i = 1; i < a.size(); ++i)
        for (std::size_t j = 1; j < b.size() && j <= n / i; ++j)
            detail::arithmetic_add_assign(result[i * j], detail::arithmetic_multiply(a[i], b[j]));
    return result;
}
} // namespace cp
