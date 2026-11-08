#pragma once
#include <cstdint>
#include <limits>
#include <stdexcept>

namespace cp {
// Exact reduction of any uint64 by a positive uint32 modulus.
class BarrettReduction {
    std::uint32_t modulus_;
    std::uint64_t reciprocal_;
public:
    explicit BarrettReduction(std::uint32_t modulus) : modulus_(modulus), reciprocal_(0) {
        if (!modulus) throw std::invalid_argument("Barrett modulus must be positive");
        if (modulus > 1) reciprocal_ = std::numeric_limits<std::uint64_t>::max() / modulus + 1;
    }
    std::uint32_t mod() const { return modulus_; }
    std::uint32_t reduce(std::uint64_t value) const {
        if (modulus_ == 1) return 0;
        const auto quotient = static_cast<std::uint64_t>((__uint128_t(value) * reciprocal_) >> 64);
        const __uint128_t product = __uint128_t(quotient) * modulus_;
        // ceil(2^64/m) overestimates the true quotient by at most one.
        return static_cast<std::uint32_t>(product > value ?
            __uint128_t(value) + modulus_ - product : __uint128_t(value) - product);
    }
    std::uint32_t multiply(std::uint32_t a, std::uint32_t b) const {
        return reduce(std::uint64_t(a) * b);
    }
};
} // namespace cp
