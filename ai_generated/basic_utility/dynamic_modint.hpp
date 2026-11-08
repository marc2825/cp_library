#pragma once
#include <cstdint>
#include <limits>
#include <stdexcept>
#include "barrett_reduction.hpp"
#include "detail/modint_operators.hpp"

namespace cp {
// Each Id has its own modulus. Every set_mod invalidates all existing values.
template <int Id = 0>
class DynamicModInt : public detail::ModIntOperators<DynamicModInt<Id>> {
    inline static BarrettReduction reducer_{998244353};
    inline static std::uint64_t generation_ = 0;
    std::uint32_t value_ = 0;
    std::uint64_t generation_at_creation_ = generation_;
    void check() const {
        if (generation_at_creation_ != generation_)
            throw std::logic_error("dynamic modint used after set_mod");
    }
public:
    DynamicModInt() = default;
    template <class T, std::enable_if_t<detail::is_integer_v<T>, int> = 0>
    DynamicModInt(T value) : value_(static_cast<std::uint32_t>(detail::normalize_modint(value, mod()))) {}
    static std::uint32_t mod() { return reducer_.mod(); }
    static void set_mod(std::uint32_t modulus) {
        BarrettReduction next(modulus);
        if (generation_ == std::numeric_limits<std::uint64_t>::max())
            throw std::overflow_error("dynamic modint generation exhausted");
        reducer_ = next;
        ++generation_;
    }
    std::uint32_t val() const { check(); return value_; }
    DynamicModInt& operator+=(const DynamicModInt& rhs) {
        check(); rhs.check();
        const std::uint64_t sum = std::uint64_t(value_) + rhs.value_;
        value_ = static_cast<std::uint32_t>(sum >= mod() ? sum - mod() : sum);
        return *this;
    }
    DynamicModInt& operator-=(const DynamicModInt& rhs) {
        check(); rhs.check();
        const std::uint64_t difference = std::uint64_t(value_) + mod() - rhs.value_;
        value_ = static_cast<std::uint32_t>(difference >= mod() ? difference - mod() : difference);
        return *this;
    }
    DynamicModInt& operator*=(const DynamicModInt& rhs) {
        check(); rhs.check();
        value_ = reducer_.multiply(value_, rhs.value_);
        return *this;
    }
};
template <int Id = 0> using dynamic_modint = DynamicModInt<Id>;
} // namespace cp
