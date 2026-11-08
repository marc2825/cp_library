#pragma once
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <random>
#include <tuple>
#include <type_traits>
#include <utility>
#include "detail/integer_traits.hpp"
#include "../heuristic_search/random.hpp"

namespace cp {
namespace seeded_hash_detail {
template <class T, class = void> struct tuple_like : std::false_type {};
template <class T> struct tuple_like<T, std::void_t<decltype(std::tuple_size<T>::value)>> : std::true_type {};
template <class T, class = void> struct sequence : std::false_type {};
template <class T> struct sequence<T, std::void_t<decltype(std::begin(std::declval<const T&>())),
    decltype(std::end(std::declval<const T&>()))>> : std::true_type {};
template <class> inline constexpr bool unsupported = false;
} // namespace seeded_hash_detail

// Stateless per-key hashing with an explicit, reproducible seed. Not cryptographic.
class SeededHash {
    std::uint64_t seed_;
    static std::uint64_t mix(std::uint64_t x) { return Random(x)(); }
    template <class T> std::uint64_t hash_value(const T& value) const {
        if constexpr (detail::is_integer_v<T> || std::is_same_v<T, bool>) {
            if constexpr (std::is_same_v<T, char>) return mix(static_cast<unsigned char>(value) ^ seed_);
            else if constexpr (sizeof(T) <= 8) return mix(static_cast<std::uint64_t>(value) ^ seed_);
            else {
                const auto bits = static_cast<__uint128_t>(value);
                return mix(mix(static_cast<std::uint64_t>(bits) ^ seed_) ^ static_cast<std::uint64_t>(bits >> 64));
            }
        } else if constexpr (std::is_enum_v<T>) {
            return hash_value(static_cast<std::underlying_type_t<T>>(value));
        } else if constexpr (seeded_hash_detail::tuple_like<T>::value) {
            std::uint64_t state = mix(seed_ ^ UINT64_C(0x7465706c65));
            std::apply([&](const auto&... fields) { ((state = mix(state ^ hash_value(fields))), ...); }, value);
            return mix(state ^ std::tuple_size<T>::value);
        } else if constexpr (seeded_hash_detail::sequence<T>::value) {
            std::uint64_t state = mix(seed_ ^ UINT64_C(0x736571)), length = 0;
            for (const auto& element : value) { state = mix(state ^ hash_value(element)); ++length; }
            return mix(state ^ length);
        } else {
            static_assert(seeded_hash_detail::unsupported<T>, "SeededHash needs integers, tuples or sequences thereof");
        }
    }
public:
    static std::uint64_t random_seed() {
        std::random_device source;
        std::uint64_t result = 0;
        for (int i = 0; i < 4; ++i) result = mix(result ^ static_cast<std::uint64_t>(source()));
        return result;
    }
    explicit SeededHash(std::uint64_t seed = random_seed()) : seed_(seed) {}
    std::uint64_t seed() const { return seed_; }
    template <class T> std::uint64_t hash64(const T& value) const { return hash_value(value); }
    template <class T> std::size_t operator()(const T& value) const { return static_cast<std::size_t>(hash64(value)); }
};
} // namespace cp
