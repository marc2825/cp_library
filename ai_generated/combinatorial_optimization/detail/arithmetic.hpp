#pragma once
#include <algorithm>
#include <limits>
#include <stdexcept>
#include <string>
#include <type_traits>

namespace cp {
// GCC/Clang extension; potentials may exceed the 64-bit input cost range.
__extension__ typedef __int128 OptimizationWide;
namespace optimization_detail {
using Wide = OptimizationWide;
__extension__ typedef unsigned __int128 UWide;
inline Wide add(Wide a, Wide b) {
    Wide r;
    if (__builtin_add_overflow(a, b, &r)) throw std::overflow_error("optimization addition overflow");
    return r;
}
inline Wide sub(Wide a, Wide b) {
    Wide r;
    if (__builtin_sub_overflow(a, b, &r)) throw std::overflow_error("optimization subtraction overflow");
    return r;
}
inline Wide mul(Wide a, Wide b) {
    Wide r;
    if (__builtin_mul_overflow(a, b, &r)) throw std::overflow_error("optimization multiplication overflow");
    return r;
}
template<class T> T narrow(Wide value) {
    static_assert(std::is_integral_v<T> && !std::is_same_v<T, bool> && sizeof(T) <= 8,
                  "an integer type of at most 64 bits is required");
    if (value < Wide(std::numeric_limits<T>::lowest()) || value > Wide(std::numeric_limits<T>::max()))
        throw std::overflow_error("optimization result does not fit its type");
    return static_cast<T>(value);
}
} // namespace optimization_detail
inline std::string optimization_to_string(OptimizationWide value) {
    using U = optimization_detail::UWide;
    const bool negative = value < 0;
    U x = negative ? U(-(value + 1)) + 1 : U(value);
    std::string text;
    do { text.push_back(char('0' + x % 10)); x /= 10; } while (x);
    if (negative) text.push_back('-');
    std::reverse(text.begin(), text.end());
    return text;
}
} // namespace cp
