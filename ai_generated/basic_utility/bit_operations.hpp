#pragma once
#include <cstdint>
#include <iterator>
#include <limits>
#include <optional>
#include <stdexcept>

namespace cp {
// Bit indices, not masks. Both return -1 for zero.
inline int topbit(std::uint64_t value) { return value ? 63 - __builtin_clzll(value) : -1; }
inline int lowbit(std::uint64_t value) { return value ? __builtin_ctzll(value) : -1; }
inline int popcount(std::uint64_t value) { return __builtin_popcountll(value); }
inline std::optional<std::uint64_t> next_same_popcount(std::uint64_t value) {
    if (!value) return std::nullopt;
    const std::uint64_t lowest = value & (~value + 1), raised = value + lowest;
    if (!raised) return std::nullopt;
    return raised | (((raised ^ value) / lowest) >> 2);
}
// Enumerate n-bit masks with exactly k set bits, in ascending numeric order.
class FixedPopcountMasks {
    int n_, k_;
public:
    class Iterator {
        std::uint64_t value_ = 0;
        int n_ = 0;
        bool done_ = true;
    public:
        using value_type = std::uint64_t;
        using difference_type = std::ptrdiff_t;
        using pointer = void;
        using reference = value_type;
        using iterator_category = std::input_iterator_tag;
        Iterator() = default;
        Iterator(std::uint64_t value, int n) : value_(value), n_(n), done_(false) {}
        value_type operator*() const { return value_; }
        Iterator& operator++() {
            const auto next = next_same_popcount(value_);
            if (!next || (n_ < 64 && *next >= (UINT64_C(1) << n_))) done_ = true;
            else value_ = *next;
            return *this;
        }
        Iterator operator++(int) { auto old = *this; ++*this; return old; }
        friend bool operator==(const Iterator& a, const Iterator& b) {
            return a.done_ == b.done_ && (a.done_ || (a.n_ == b.n_ && a.value_ == b.value_));
        }
        friend bool operator!=(const Iterator& a, const Iterator& b) { return !(a == b); }
    };
    FixedPopcountMasks(int n, int k) : n_(n), k_(k) {
        if (n < 0 || n > 64 || k < 0 || k > n) throw std::invalid_argument("require 0 <= k <= n <= 64");
    }
    Iterator begin() const {
        return Iterator(k_ == 64 ? std::numeric_limits<std::uint64_t>::max() : (UINT64_C(1) << k_) - 1, n_);
    }
    Iterator end() const { return Iterator(); }
};
} // namespace cp
