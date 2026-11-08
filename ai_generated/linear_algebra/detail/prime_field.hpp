#pragma once
#include <stdexcept>
#include "../../linear_recurrence/detail/common.hpp"

namespace cp { namespace linear_algebra_detail {
// 素数法の連立方程式用。整数だけのAPIへ変換する内部型。
template <int Mod> struct PrimeField {
    using R = linear_recurrence_detail::Arithmetic<Mod>;
    static_assert(linear_recurrence_detail::prime(Mod), "Gauss-Jordan modulus must be prime");
    int value;
    PrimeField(long long x = 0) : value(R::norm(x)) {}
    PrimeField& operator+=(const PrimeField& other) { value = R::add(value, other.value); return *this; }
    PrimeField& operator-=(const PrimeField& other) { value = R::sub(value, other.value); return *this; }
    friend PrimeField operator*(const PrimeField& a, const PrimeField& b) { return R::mul(a.value, b.value); }
    friend PrimeField operator/(const PrimeField& a, const PrimeField& b) {
        if (!b.value) throw std::domain_error("division by zero in prime field");
        return R::mul(a.value, R::inverse(b.value));
    }
    friend bool operator==(const PrimeField& a, const PrimeField& b) { return a.value == b.value; }
};
} } // namespace cp::linear_algebra_detail
