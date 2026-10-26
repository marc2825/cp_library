#pragma once
#include <cstdint>
#include <numeric>
#include <optional>
#include <stdexcept>
#include "../number_theory_convolution/pollard_rho.hpp"

namespace cp {
// uint64 全域。lambda(1)=1。素因数分解は既存 Pollard--Rho を共有。
inline std::uint64_t carmichael_lambda(std::uint64_t modulus) {
    if (!modulus) throw std::invalid_argument("zero modulus");
    std::uint64_t result = 1;
    for (auto [p, e] : prime_factorization_64(modulus)) {
        std::uint64_t value = p - 1;
        for (int i = 1; i < e; ++i) value *= p;
        if (p == 2 && e >= 3) value /= 2;
        result = result / std::gcd(result, value) * value;
    }
    return result;
}
inline std::optional<std::uint64_t> multiplicative_order(std::uint64_t a, std::uint64_t modulus) {
    if (!modulus) throw std::invalid_argument("zero modulus");
    if (std::gcd(a, modulus) != 1) return std::nullopt;
    auto order = carmichael_lambda(modulus);
    for (auto [p, e] : prime_factorization_64(order)) {
        (void)e;
        while (order % p == 0 && factorization_detail::power(a % modulus, order / p, modulus) == 1 % modulus)
            order /= p;
    }
    return order;
}
} // namespace cp
