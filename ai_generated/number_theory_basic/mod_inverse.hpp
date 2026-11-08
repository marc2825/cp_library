#pragma once
#include <optional>
#include <stdexcept>
#include "extended_gcd.hpp"

namespace cp {
// 素数でなくても可。逆元なしは nullopt、modulus=1 の代表元は0。
inline std::optional<long long> mod_inverse(long long a, long long modulus) {
    if (modulus <= 0) throw std::invalid_argument("modulus must be positive");
    const auto result = extended_gcd(a, modulus);
    if (result.gcd != 1) return std::nullopt;
    const auto r = result.x % modulus;
    return r < 0 ? r + modulus : r;
}
} // namespace cp
