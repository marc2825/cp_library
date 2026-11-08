#pragma once
#include <cmath>
#include <cstddef>
#include <functional>
#include <limits>
#include <stdexcept>
#include <type_traits>

namespace cp {
enum class SearchGoal { minimize, maximize };
enum class SearchTermination { iteration_limit, depth_limit, local_optimum, exhausted, stopped };
struct SearchOptions {
    std::size_t max_iterations = 1000;
    SearchGoal goal = SearchGoal::minimize;
    std::function<bool()> stop; // Optional. true requests a cooperative stop.
};
template<class State, class Score> struct SearchResult {
    State best_state, current_state;
    Score best_score, current_score;
    std::size_t iterations = 0, evaluations = 1, accepted = 0;
    SearchTermination termination = SearchTermination::iteration_limit;
};
namespace heuristic_detail {
inline void validate_goal(SearchGoal goal) {
    if (goal != SearchGoal::minimize && goal != SearchGoal::maximize)
        throw std::invalid_argument("invalid search goal");
}
template<class Score> void validate_score(Score score) {
    static_assert(std::is_arithmetic_v<Score> && !std::is_same_v<Score, bool> && sizeof(Score) <= 8,
                  "Search needs a numeric score of at most 64 bits, other than bool");
    if constexpr (std::is_floating_point_v<Score>)
        if (!std::isfinite(score)) throw std::invalid_argument("nonfinite search score");
}
template<class Score> bool better(Score a, Score b, SearchGoal goal) {
    return goal == SearchGoal::minimize ? a < b : a > b;
}
inline void count_evaluation(std::size_t& count) {
    if (count == std::numeric_limits<std::size_t>::max()) throw std::overflow_error("search evaluation count");
    ++count;
}
// Called only for a strictly worse candidate. Subtract integers before converting
// so adjacent large scores and signed extremes keep the correct difference.
template<class Score> long double worsening_loss(Score current, Score candidate, SearchGoal goal) {
    const Score high = goal == SearchGoal::minimize ? candidate : current;
    const Score low = goal == SearchGoal::minimize ? current : candidate;
    if constexpr (std::is_integral_v<Score>) {
        using U = std::make_unsigned_t<Score>;
        return static_cast<long double>(static_cast<U>(static_cast<U>(high) - static_cast<U>(low)));
    } else return static_cast<long double>(high) - static_cast<long double>(low);
}
template<class Evaluate, class State>
using ScoreOf = std::decay_t<std::invoke_result_t<Evaluate&, const State&>>;
} // namespace heuristic_detail
} // namespace cp
