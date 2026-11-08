#pragma once
#include <cstdint>
#include "detail/modint_operators.hpp"

namespace cp {
// Arithmetic modulo 2^61-1. Multiplication folds the high bits without division.
class ModInt61 : public detail::ModIntOperators<ModInt61> {
    std::uint64_t value_ = 0;
public:
    ModInt61() = default;
    template <class T, std::enable_if_t<detail::is_integer_v<T>, int> = 0>
    ModInt61(T value) : value_(detail::normalize_modint(value, mod())) {}
    static constexpr std::uint64_t mod() { return (std::uint64_t(1) << 61) - 1; }
    std::uint64_t val() const { return value_; }
    ModInt61& operator+=(const ModInt61& rhs) {
        value_ += rhs.value_;
        if (value_ >= mod()) value_ -= mod();
        return *this;
    }
    ModInt61& operator-=(const ModInt61& rhs) {
        value_ = value_ >= rhs.value_ ? value_ - rhs.value_ : mod() - (rhs.value_ - value_);
        return *this;
    }
    ModInt61& operator*=(const ModInt61& rhs) {
        const __uint128_t product = __uint128_t(value_) * rhs.value_;
        value_ = (static_cast<std::uint64_t>(product) & mod()) + static_cast<std::uint64_t>(product >> 61);
        if (value_ >= mod()) value_ -= mod();
        return *this;
    }
};
} // namespace cp
