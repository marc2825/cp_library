#pragma once
#include <cstdint>
#include <limits>
#include <type_traits>

namespace cp {
namespace detail {
template <class T>
inline constexpr bool is_integer_v =
    (std::is_integral<T>::value || std::is_same<T, __int128_t>::value ||
     std::is_same<T, __uint128_t>::value) && !std::is_same<T, bool>::value;
template <class T> struct unsigned_integer { using type = std::make_unsigned_t<T>; };
template <> struct unsigned_integer<__int128_t> { using type = __uint128_t; };
template <> struct unsigned_integer<__uint128_t> { using type = __uint128_t; };
template <class T> using unsigned_integer_t = typename unsigned_integer<T>::type;

template <class T, std::enable_if_t<is_integer_v<T>, int> = 0>
inline std::uint64_t normalize_modint(T value, std::uint64_t modulus) {
    if constexpr (std::numeric_limits<T>::is_signed) {
        const auto remainder = static_cast<__int128_t>(value) % modulus;
        return static_cast<std::uint64_t>(remainder < 0 ? remainder + modulus : remainder);
    } else {
        return static_cast<std::uint64_t>(static_cast<__uint128_t>(value) % modulus);
    }
}
} // namespace detail
} // namespace cp
