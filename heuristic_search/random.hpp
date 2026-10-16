#pragma once
#include <algorithm>
#include <cstdint>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <type_traits>

namespace cp {
// SplitMix64. Algorithm/constants: Sebastiano Vigna, public domain,
// https://prng.di.unimi.it/splitmix64.c (2015). Fixed default seed; no global state.
class Random {
    std::uint64_t state_;
public:
    using result_type = std::uint64_t;
    explicit Random(std::uint64_t seed_value = 0) : state_(seed_value) {}
    void seed(std::uint64_t value = 0) { state_ = value; }
    static constexpr result_type min() { return 0; }
    static constexpr result_type max() { return UINT64_MAX; }
    result_type operator()() {
        std::uint64_t z = (state_ += UINT64_C(0x9e3779b97f4a7c15));
        z = (z ^ (z >> 30)) * UINT64_C(0xbf58476d1ce4e5b9);
        z = (z ^ (z >> 27)) * UINT64_C(0x94d049bb133111eb);
        return z ^ (z >> 31);
    }
    // Uniform on [0, bound). Rejection removes modulo bias; bound must be positive.
    result_type uniform(result_type bound) {
        if (!bound) throw std::invalid_argument("random bound must be positive");
        const result_type threshold = (result_type(0) - bound) % bound;
        for (;;) {
            const auto value = (*this)();
            if (value >= threshold) return value % bound;
        }
    }
    // Inclusive [lo, hi], including full signed/unsigned 64-bit ranges.
    template<class Int> Int integer(Int lo, Int hi) {
        static_assert(std::is_integral_v<Int> && !std::is_same_v<Int, bool> && sizeof(Int) <= 8,
                      "Random::integer needs an integer of at most 64 bits, other than bool");
        if (hi < lo) throw std::invalid_argument("reversed random interval");
        using U = std::make_unsigned_t<Int>;
        const auto span = static_cast<result_type>(static_cast<U>(static_cast<U>(hi) - static_cast<U>(lo))) + 1;
        const U encoded = static_cast<U>(static_cast<U>(lo) + static_cast<U>(span ? uniform(span) : (*this)()));
        if constexpr (std::is_signed_v<Int>) {
            if (encoded > static_cast<U>(std::numeric_limits<Int>::max()))
                return static_cast<Int>(-1 - static_cast<Int>(static_cast<U>(std::numeric_limits<U>::max() - encoded)));
        }
        return static_cast<Int>(encoded);
    }
    // Exactly one raw draw, using its upper 53 bits. Uniform grid in [0, 1).
    double real() { return static_cast<double>((*this)() >> 11) * 0x1.0p-53; }
    template<class RandomAccessIterator> void shuffle(RandomAccessIterator first, RandomAccessIterator last) {
        using Difference = typename std::iterator_traits<RandomAccessIterator>::difference_type;
        const Difference n = last - first;
        if (n < 0) throw std::invalid_argument("reversed shuffle interval");
        for (Difference i = n; i > 1; --i)
            std::iter_swap(first + (i - 1), first + static_cast<Difference>(uniform(static_cast<result_type>(i))));
    }
};
} // namespace cp
