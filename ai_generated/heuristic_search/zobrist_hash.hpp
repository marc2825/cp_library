#pragma once
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <type_traits>
#include <vector>
#include "random.hpp"

namespace cp {
// 位置×値の固定seed 64bit表。XORで1位置の変更を更新。衝突あり。
// docs/heuristic_search/zobrist_hash.md
class ZobristHash {
    std::size_t positions_, values_;
    std::vector<std::uint64_t> keys_;
public:
    ZobristHash(std::size_t positions, std::size_t values, std::uint64_t seed = 0) : positions_(positions), values_(values) {
        if (!values) throw std::invalid_argument("Zobrist values must be positive");
        if (positions > keys_.max_size() / values) throw std::length_error("Zobrist table too large");
        keys_.resize(positions * values); Random random(seed); for (auto& key : keys_) key = random();
    }
    std::size_t positions() const { return positions_; }
    std::size_t values() const { return values_; }
    std::uint64_t token(std::size_t position, std::size_t value) const {
        if (position >= positions_ || value >= values_) throw std::out_of_range("Zobrist position or value");
        return keys_[position * values_ + value];
    }
    template <class Int> std::uint64_t hash(const std::vector<Int>& state) const {
        static_assert(std::is_integral_v<Int> && !std::is_same_v<Int, bool> && sizeof(Int) <= sizeof(std::size_t), "Zobrist state needs non-bool integers fitting size_t");
        if (state.size() != positions_) throw std::invalid_argument("Zobrist state size differs");
        std::uint64_t result = 0;
        for (std::size_t i = 0; i < positions_; ++i) {
            if constexpr (std::is_signed_v<Int>) if (state[i] < 0) throw std::out_of_range("negative Zobrist value");
            result ^= token(i, static_cast<std::size_t>(state[i]));
        }
        return result;
    }
    std::uint64_t change(std::uint64_t hash, std::size_t position, std::size_t old_value, std::size_t new_value) const {
        return hash ^ token(position, old_value) ^ token(position, new_value);
    }
};
} // namespace cp
