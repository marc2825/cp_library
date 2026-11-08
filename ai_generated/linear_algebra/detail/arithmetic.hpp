#pragma once
#include <cmath>
#include <stdexcept>
#include <type_traits>
#include "../../number_theory_convolution/detail/arithmetic.hpp"

namespace cp { namespace linear_algebra_detail {
template <class T> void input(const T& x) {
    if constexpr (std::is_floating_point_v<T>) if (!std::isfinite(x)) throw std::invalid_argument("matrix input must be finite");
}
template <class T> T finite(T x) {
    if constexpr (std::is_floating_point_v<T>) if (!std::isfinite(x)) throw std::overflow_error("matrix arithmetic became nonfinite");
    return x;
}
template <class T> T multiply(const T& a, const T& b) { return finite<T>(detail::arithmetic_multiply(a, b)); }
template <class T> void add(T& a, const T& b) { detail::arithmetic_add_assign(a, b); a = finite<T>(a); }
template <class T> void subtract(T& a, const T& b) { detail::arithmetic_subtract_assign(a, b); a = finite<T>(a); }
} } // namespace cp::linear_algebra_detail
