#pragma once
#include <cstdint>
#include <stdexcept>
#include "detail/modint_operators.hpp"

namespace cp {
// R=2^64, odd 1 <= modulus < 2^63. Encoded residues are canonical in [0,m).
class MontgomeryReduction64 {
    std::uint64_t modulus_, negative_inverse_, r_squared_;
    std::uint64_t reduce_unchecked(__uint128_t value) const {
        const std::uint64_t multiple = static_cast<std::uint64_t>(value) * negative_inverse_;
        const auto result = static_cast<std::uint64_t>((value + __uint128_t(multiple) * modulus_) >> 64);
        return result >= modulus_ ? result - modulus_ : result;
    }
public:
    explicit MontgomeryReduction64(std::uint64_t modulus)
        : modulus_(modulus), negative_inverse_(1), r_squared_(0) {
        if (!(modulus & 1) || modulus >= (std::uint64_t(1) << 63))
            throw std::invalid_argument("Montgomery modulus must be odd and below 2^63");
        for (int i = 0; i < 6; ++i) negative_inverse_ *= 2 - modulus * negative_inverse_;
        negative_inverse_ = 0 - negative_inverse_;
        const auto r = static_cast<std::uint64_t>((__uint128_t(1) << 64) % modulus);
        r_squared_ = static_cast<std::uint64_t>(__uint128_t(r) * r % modulus);
    }
    std::uint64_t mod() const { return modulus_; }
    std::uint64_t reduce(__uint128_t value) const {
        if (value >= (__uint128_t(modulus_) << 64))
            throw std::out_of_range("Montgomery input must be below modulus * 2^64");
        return reduce_unchecked(value);
    }
    std::uint64_t encode(std::uint64_t value) const {
        return reduce_unchecked(__uint128_t(value % modulus_) * r_squared_);
    }
    std::uint64_t decode(std::uint64_t value) const {
        if (value >= modulus_) throw std::out_of_range("noncanonical Montgomery residue");
        return reduce_unchecked(value);
    }
    std::uint64_t multiply_encoded(std::uint64_t a, std::uint64_t b) const {
        if (a >= modulus_ || b >= modulus_) throw std::out_of_range("noncanonical Montgomery residue");
        return reduce_unchecked(__uint128_t(a) * b);
    }
};

template <std::uint64_t Mod>
class MontgomeryModInt64 : public detail::ModIntOperators<MontgomeryModInt64<Mod>> {
    static_assert((Mod & 1) && Mod < (std::uint64_t(1) << 63), "odd modulus below 2^63 required");
    inline static const MontgomeryReduction64 reducer_{Mod};
    std::uint64_t value_ = 0;
public:
    MontgomeryModInt64() = default;
    template <class T, std::enable_if_t<detail::is_integer_v<T>, int> = 0>
    MontgomeryModInt64(T value) : value_(reducer_.encode(detail::normalize_modint(value, Mod))) {}
    static constexpr std::uint64_t mod() { return Mod; }
    std::uint64_t val() const { return reducer_.decode(value_); }
    MontgomeryModInt64& operator+=(const MontgomeryModInt64& rhs) {
        value_ += rhs.value_;
        if (value_ >= Mod) value_ -= Mod;
        return *this;
    }
    MontgomeryModInt64& operator-=(const MontgomeryModInt64& rhs) {
        value_ = value_ >= rhs.value_ ? value_ - rhs.value_ : Mod - (rhs.value_ - value_);
        return *this;
    }
    MontgomeryModInt64& operator*=(const MontgomeryModInt64& rhs) {
        value_ = reducer_.multiply_encoded(value_, rhs.value_);
        return *this;
    }
};
} // namespace cp
