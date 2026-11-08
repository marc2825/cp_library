#pragma once
#include <cstddef>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <type_traits>
#include <vector>

namespace cp {
struct NimMove {
    std::size_t pile;
    std::uint64_t remaining, removed;
};
namespace game_detail {
struct NimSummary {
    std::uint64_t xor_sum = 0;
    std::size_t ones = 0, large = 0, one_index = 0, large_index = 0;
};
template<class Int> NimSummary nim_summary(const std::vector<Int>& piles) {
    static_assert(std::is_integral_v<Int> && !std::is_same_v<Int, bool> && sizeof(Int) <= 8,
                  "Nim needs integer piles of at most 64 bits, other than bool");
    NimSummary result;
    for (std::size_t i = 0; i < piles.size(); ++i) {
        if constexpr (std::is_signed_v<Int>)
            if (piles[i] < 0) throw std::invalid_argument("Nim pile size must be nonnegative");
        const auto size = static_cast<std::uint64_t>(piles[i]);
        result.xor_sum ^= size;
        if (size == 1) { ++result.ones; result.one_index = i; }
        if (size > 1) { ++result.large; result.large_index = i; }
    }
    return result;
}
template<class Int> std::optional<NimMove> normal_move(const std::vector<Int>& piles, std::uint64_t sum) {
    if (!sum) return std::nullopt;
    for (std::size_t i = 0; i < piles.size(); ++i) {
        const auto size = static_cast<std::uint64_t>(piles[i]), target = size ^ sum;
        if (target < size) return NimMove{i, target, size - target};
    }
    return std::nullopt;
}
} // namespace game_detail
// Normal play: taking the last stone wins; an empty position loses.
template<class Int> std::uint64_t nim_sum(const std::vector<Int>& piles) {
    return game_detail::nim_summary(piles).xor_sum;
}
template<class Int> bool nim_winning(const std::vector<Int>& piles) { return nim_sum(piles) != 0; }
template<class Int> std::optional<NimMove> nim_winning_move(const std::vector<Int>& piles) {
    const auto state = game_detail::nim_summary(piles);
    return game_detail::normal_move(piles, state.xor_sum);
}
} // namespace cp
