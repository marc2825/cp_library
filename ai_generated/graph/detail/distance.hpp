#pragma once

#include <cmath>
#include <limits>
#include <stdexcept>
#include <type_traits>

namespace cp {

// 最大値は到達不能専用。有限な距離とその計算途中の値は最大値未満であること。
template <class T>
constexpr T distance_inf() { return std::numeric_limits<T>::max(); }

namespace detail {

template <class T>
T distance_add(T a, T b) {
    static_assert(std::is_arithmetic_v<T>, "distance type must be arithmetic");
    if constexpr (std::is_integral_v<T>) {
        if (b > 0 && a >= std::numeric_limits<T>::max() - b)
            throw std::overflow_error("distance exceeds finite range");
        if constexpr (std::is_signed_v<T>) {
            if (b < 0 && a < std::numeric_limits<T>::lowest() - b)
                throw std::overflow_error("distance exceeds finite range");
        }
    }
    T value = a + b;
    if constexpr (std::is_floating_point_v<T>) {
        if (!std::isfinite(value) || value >= distance_inf<T>())
            throw std::overflow_error("distance exceeds finite range");
    }
    return value;
}

template <class T>
T distance_subtract(T a, T b) {
    if constexpr (std::is_integral_v<T>) {
        if (b > 0 && a < std::numeric_limits<T>::lowest() + b)
            throw std::overflow_error("distance exceeds finite range");
        if constexpr (std::is_signed_v<T>) {
            if (b < 0 && a >= std::numeric_limits<T>::max() + b)
                throw std::overflow_error("distance exceeds finite range");
        }
    }
    T value = a - b;
    if constexpr (std::is_floating_point_v<T>) {
        if (!std::isfinite(value) || value >= distance_inf<T>())
            throw std::overflow_error("distance exceeds finite range");
    }
    return value;
}

} // namespace detail
} // namespace cp
