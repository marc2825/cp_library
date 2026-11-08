#pragma once
#include <limits>
#include <optional>
#include <stdexcept>
#include "detail/integer_traits.hpp"

namespace cp {
// Built-in integral types except bool, including signed/unsigned 128-bit integers.
template <class T> std::optional<T> checked_add(T a, T b) {
    static_assert(detail::is_integer_v<T>, "integer required");
    T result;
    if (__builtin_add_overflow(a, b, &result)) return std::nullopt;
    return result;
}
template <class T> std::optional<T> checked_sub(T a, T b) {
    static_assert(detail::is_integer_v<T>, "integer required");
    T result;
    if (__builtin_sub_overflow(a, b, &result)) return std::nullopt;
    return result;
}
template <class T> std::optional<T> checked_mul(T a, T b) {
    static_assert(detail::is_integer_v<T>, "integer required");
    T result;
    if (__builtin_mul_overflow(a, b, &result)) return std::nullopt;
    return result;
}
template <class T> T saturating_add(T a, T b) {
    if (const auto result = checked_add(a, b)) return *result;
    return b < 0 ? std::numeric_limits<T>::lowest() : std::numeric_limits<T>::max();
}
template <class T> T saturating_sub(T a, T b) {
    if (const auto result = checked_sub(a, b)) return *result;
    return b < 0 ? std::numeric_limits<T>::max() : std::numeric_limits<T>::lowest();
}
template <class T> T saturating_mul(T a, T b) {
    if (const auto result = checked_mul(a, b)) return *result;
    return (a < 0) != (b < 0) ? std::numeric_limits<T>::lowest() : std::numeric_limits<T>::max();
}
// Nonnegative inputs. Test the mathematical sum/product without overflowing T.
template <class T> bool add_leq(T a, T b, T limit) {
    static_assert(detail::is_integer_v<T>, "integer required");
    if (a < 0 || b < 0 || limit < 0) throw std::invalid_argument("nonnegative inputs required");
    return a <= limit && b <= limit - a;
}
template <class T> bool multiply_leq(T a, T b, T limit) {
    static_assert(detail::is_integer_v<T>, "integer required");
    if (a < 0 || b < 0 || limit < 0) throw std::invalid_argument("nonnegative inputs required");
    return b == 0 || a <= limit / b;
}
// min(a+b, infinity), with infinity absorbing, for 0 <= a,b <= infinity.
template <class T> T infinity_add(T a, T b, T infinity) {
    static_assert(detail::is_integer_v<T>, "integer required");
    if (a < 0 || b < 0 || infinity < 0 || a > infinity || b > infinity)
        throw std::invalid_argument("invalid infinity addition operands");
    return a >= infinity - b ? infinity : static_cast<T>(a + b);
}
namespace detail {
template <class T> void check_integer_division(T a, T b) {
    static_assert(is_integer_v<T>, "integer required");
    if (b == 0) throw std::invalid_argument("division by zero");
    if constexpr (std::numeric_limits<T>::is_signed) {
        if (a == std::numeric_limits<T>::lowest() && b == -1)
            throw std::overflow_error("integer quotient out of range");
    }
}
} // namespace detail
template <class T> T floor_div(T a, T b) {
    detail::check_integer_division(a, b);
    const T q = a / b, r = a % b;
    return static_cast<T>(q - (r != 0 && ((a < 0) != (b < 0))));
}
template <class T> T ceil_div(T a, T b) {
    detail::check_integer_division(a, b);
    const T q = a / b, r = a % b;
    return static_cast<T>(q + (r != 0 && ((a < 0) == (b < 0))));
}
} // namespace cp
