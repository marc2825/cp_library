#pragma once
#include <cstdint>
#include <numeric>
#include <optional>
#include <stdexcept>
#include <vector>
#include "detail/advanced_arithmetic.hpp"

namespace cp {
// 可換環。inverse(product) は optional<T> を返す。空入力では呼ばない。
template<class T, class Inverse>
std::optional<std::vector<T>> batch_inverse(const std::vector<T>& values, Inverse inverse) {
    if (values.empty()) return std::vector<T>{};
    std::vector<T> prefix(values.size() + 1, T(1)), result(values.size());
    for (std::size_t i = 0; i < values.size(); ++i) prefix[i + 1] = prefix[i] * values[i];
    auto current = inverse(prefix.back());
    if (!current) return std::nullopt;
    for (std::size_t i = values.size(); i-- > 0;) {
        result[i] = *current * prefix[i];
        *current = *current * values[i];
    }
    return result;
}
inline std::optional<std::vector<std::uint64_t>> batch_inverse_mod(
        const std::vector<std::uint64_t>& values, std::uint64_t modulus) {
    if (!modulus) throw std::invalid_argument("zero modulus");
    std::vector<std::uint64_t> prefix(values.size() + 1, 1 % modulus), result(values.size());
    if (values.empty()) return result;
    for (std::size_t i = 0; i < values.size(); ++i) prefix[i + 1] = nt_detail::mul(prefix[i], values[i] % modulus, modulus);
    if (std::gcd(prefix.back(), modulus) != 1) return std::nullopt;
    auto current = nt_detail::inverse(prefix.back(), modulus);
    for (std::size_t i = values.size(); i-- > 0;) {
        result[i] = nt_detail::mul(current, prefix[i], modulus);
        current = nt_detail::mul(current, values[i] % modulus, modulus);
    }
    return result;
}
} // namespace cp
