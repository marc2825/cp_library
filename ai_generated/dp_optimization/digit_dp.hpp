#pragma once
#include <array>
#include <cassert>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>
#include "detail/arithmetic.hpp"

namespace cp {
// 0<=x<=upper の十進桁DP。transition(state,digit,started,next_started)->state/-1。
// accept(state,started)->bool。先頭0も遷移へ渡す。詳細: docs/dp_optimization/digit_dp.md。
template <class Count = long long, class Transition, class Accept>
Count digit_dp(const std::string& upper, int state_count, int initial_state,
               Transition transition, Accept accept) {
    assert(state_count > 0 && 0 <= initial_state && initial_state < state_count);
    if (upper.empty() || upper.find_first_not_of("0123456789") != std::string::npos)
        throw std::invalid_argument("digit_dp requires a nonempty decimal bound");
    using Cell = std::array<std::array<Count, 2>, 2>; // [tight][started]
    const Cell zero{{{Count(0), Count(0)}, {Count(0), Count(0)}}};
    std::vector<Cell> dp(state_count, zero); dp[initial_state][1][0] = Count(1);
    for (char c : upper) {
        const int limit = c - '0'; std::vector<Cell> next(state_count, zero);
        for (int state = 0; state < state_count; ++state) for (int tight = 0; tight < 2; ++tight) for (int started = 0; started < 2; ++started) {
            if (dp[state][tight][started] == Count(0)) continue;
            for (int digit = 0; digit <= (tight ? limit : 9); ++digit) {
                const bool next_started = started || digit != 0;
                const int to = transition(state, digit, static_cast<bool>(started), next_started);
                assert(-1 <= to && to < state_count);
                if (to != -1) detail::dp_count_add(next[to][tight && digit == limit][next_started], dp[state][tight][started]);
            }
        }
        dp = std::move(next);
    }
    Count answer(0);
    for (int state = 0; state < state_count; ++state) for (int started = 0; started < 2; ++started) if (accept(state, static_cast<bool>(started)))
        for (int tight = 0; tight < 2; ++tight) detail::dp_count_add(answer, dp[state][tight][started]);
    return answer;
}
} // namespace cp
