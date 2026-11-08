#pragma once
#include <cstdint>
#include <optional>
#include <vector>
#include "nim.hpp"

namespace cp {
// Taking the last stone loses. A position with no legal move is winning by convention.
template<class Int> bool misere_nim_winning(const std::vector<Int>& piles) {
    const auto state = game_detail::nim_summary(piles);
    return state.large ? state.xor_sum != 0 : state.ones % 2 == 0;
}
template<class Int> std::optional<NimMove> misere_nim_winning_move(const std::vector<Int>& piles) {
    const auto state = game_detail::nim_summary(piles);
    if (!state.large) {
        if (!state.ones || state.ones % 2) return std::nullopt;
        return NimMove{state.one_index, 0, 1};
    }
    if (state.large == 1) {
        const std::uint64_t target = state.ones % 2 ? 0 : 1;
        const auto size = static_cast<std::uint64_t>(piles[state.large_index]);
        return NimMove{state.large_index, target, size - target};
    }
    return game_detail::normal_move(piles, state.xor_sum);
}
} // namespace cp
