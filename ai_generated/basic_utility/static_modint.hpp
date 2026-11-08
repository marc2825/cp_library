#pragma once
#include <cstdint>
#include "detail/modint_operators.hpp"

namespace cp {
// 1 <= Mod <= UINT32_MAX. Composite moduli are allowed; inv() checks existence.
template <std::uint32_t Mod>
class StaticModInt : public detail::ModIntOperators<StaticModInt<Mod>> {
    static_assert(Mod > 0, "modulus must be positive");
    std::uint32_t value_ = 0;
public:
    StaticModInt() = default;
    template <class T, std::enable_if_t<detail::is_integer_v<T>, int> = 0>
    StaticModInt(T value) : value_(static_cast<std::uint32_t>(detail::normalize_modint(value, Mod))) {}
    static constexpr std::uint32_t mod() { return Mod; }
    std::uint32_t val() const { return value_; }
    StaticModInt& operator+=(const StaticModInt& rhs) {
        const std::uint64_t sum = std::uint64_t(value_) + rhs.value_;
        value_ = static_cast<std::uint32_t>(sum >= Mod ? sum - Mod : sum);
        return *this;
    }
    StaticModInt& operator-=(const StaticModInt& rhs) {
        const std::uint64_t difference = std::uint64_t(value_) + Mod - rhs.value_;
        value_ = static_cast<std::uint32_t>(difference >= Mod ? difference - Mod : difference);
        return *this;
    }
    StaticModInt& operator*=(const StaticModInt& rhs) {
        value_ = static_cast<std::uint32_t>(std::uint64_t(value_) * rhs.value_ % Mod);
        return *this;
    }
};
template <std::uint32_t Mod> using static_modint = StaticModInt<Mod>;
using ModInt998244353 = StaticModInt<998244353>;
using ModInt1000000007 = StaticModInt<1000000007>;
} // namespace cp
