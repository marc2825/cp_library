#pragma once
#include <stdexcept>
#include <vector>
#include "fwht.hpp"
#include "subset_zeta_transform.hpp"
#include "subset_mobius_transform.hpp"
#include "superset_zeta_transform.hpp"
#include "superset_mobius_transform.hpp"

namespace cp {
namespace set_function_detail {
template <int Operation, class T> std::vector<T> bitwise_product(const std::vector<T>& a, const std::vector<T>& b) {
    validate_size(a.size());
    if (a.size() != b.size()) throw std::invalid_argument("bitwise convolution size mismatch");
    auto x = a, y = b;
    if constexpr (Operation == 0) { superset_zeta_transform(x); superset_zeta_transform(y); }
    if constexpr (Operation == 1) { subset_zeta_transform(x); subset_zeta_transform(y); }
    if constexpr (Operation == 2) { fwht(x); fwht(y); }
    for (std::size_t i = 0; i < x.size(); ++i) x[i] = detail::arithmetic_multiply(x[i], y[i]);
    if constexpr (Operation == 0) superset_mobius_transform(x);
    if constexpr (Operation == 1) subset_mobius_transform(x);
    if constexpr (Operation == 2) fwht(x, true);
    return x;
}
} // namespace set_function_detail
// 同じ正の2冪長。h[k]=sum_{i op j=k}a[i]*b[j]。O(N log N)。
// AND/ORは環、XORはさらに2で割れる係数。docs/set_function/bitwise_convolution.md
template <class T> std::vector<T> bitwise_and_convolution(const std::vector<T>& a, const std::vector<T>& b) { return set_function_detail::bitwise_product<0>(a, b); }
template <class T> std::vector<T> bitwise_or_convolution(const std::vector<T>& a, const std::vector<T>& b) { return set_function_detail::bitwise_product<1>(a, b); }
template <class T> std::vector<T> bitwise_xor_convolution(const std::vector<T>& a, const std::vector<T>& b) { return set_function_detail::bitwise_product<2>(a, b); }
} // namespace cp
