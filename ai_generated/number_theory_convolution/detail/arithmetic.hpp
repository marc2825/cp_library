#pragma once
#include <limits>
#include <stdexcept>
#include <type_traits>

namespace cp { namespace detail {
template <class T> void arithmetic_add_assign(T& a, const T& b) {
    if constexpr (std::is_integral_v<T>) {
        static_assert(!std::is_same_v<T, bool>, "bool coefficients are not supported");
        const auto hi = std::numeric_limits<T>::max(), lo = std::numeric_limits<T>::lowest();
        bool overflow;
        if constexpr (std::is_signed_v<T>) overflow = (b > 0 && a > hi - b) || (b < 0 && a < lo - b);
        else overflow = a > hi - b;
        if (overflow) throw std::overflow_error("arithmetic function addition overflow");
    }
    a += b;
}
template <class T> void arithmetic_subtract_assign(T& a, const T& b) {
    if constexpr (std::is_integral_v<T>) {
        static_assert(!std::is_same_v<T, bool>, "bool coefficients are not supported");
        const auto hi = std::numeric_limits<T>::max(), lo = std::numeric_limits<T>::lowest();
        bool overflow;
        if constexpr (std::is_signed_v<T>) overflow = (b > 0 && a < lo + b) || (b < 0 && a > hi + b);
        else overflow = a < b;
        if (overflow) throw std::overflow_error("arithmetic function subtraction overflow");
    }
    a -= b;
}
template <class T> T arithmetic_multiply(const T& a, const T& b) {
    if constexpr (std::is_integral_v<T>) {
        static_assert(!std::is_same_v<T, bool>, "bool coefficients are not supported");
        const auto hi = std::numeric_limits<T>::max(), lo = std::numeric_limits<T>::lowest();
        bool overflow = false;
        if constexpr (std::is_signed_v<T>) {
            if (a > 0) overflow = b > 0 ? a > hi / b : b < lo / a;
            else if (a < 0) {
                if (b > 0) overflow = a < lo / b;
                else if (b < 0) overflow = a < hi / b;
            }
        } else overflow = b != 0 && a > hi / b;
        if (overflow) throw std::overflow_error("arithmetic function multiplication overflow");
    }
    return a * b;
}
} } // namespace cp::detail
