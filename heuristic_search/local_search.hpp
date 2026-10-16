#pragma once
#include <cstddef>
#include <optional>
#include <stdexcept>
#include <utility>
#include "search_common.hpp"

namespace cp {
enum class LocalSearchMode { first_improvement, best_improvement };
struct LocalSearchOptions : SearchOptions {
    LocalSearchMode mode = LocalSearchMode::best_improvement;
};
// neighbors(const State&) returns a range of candidate states by value.
// Only strict improvements are accepted. A completed scan with none is a local optimum.
template<class State, class Evaluate, class Neighbors>
auto local_search(State initial, Evaluate evaluate, Neighbors neighbors, const LocalSearchOptions& options = {})
    -> SearchResult<State, heuristic_detail::ScoreOf<Evaluate, State>> {
    using Score = heuristic_detail::ScoreOf<Evaluate, State>;
    heuristic_detail::validate_goal(options.goal);
    if (options.mode != LocalSearchMode::first_improvement && options.mode != LocalSearchMode::best_improvement)
        throw std::invalid_argument("invalid local search mode");
    const Score initial_score = evaluate(std::as_const(initial));
    heuristic_detail::validate_score(initial_score);
    SearchResult<State, Score> result{initial, initial, initial_score, initial_score};
    for (std::size_t step = 0; step < options.max_iterations; ++step) {
        if (options.stop && options.stop()) { result.termination = SearchTermination::stopped; return result; }
        auto candidates = neighbors(std::as_const(result.current_state));
        std::optional<State> chosen;
        Score chosen_score = result.current_score;
        for (const auto& candidate : candidates) {
            if (options.stop && options.stop()) { result.termination = SearchTermination::stopped; return result; }
            heuristic_detail::count_evaluation(result.evaluations);
            const Score score = evaluate(candidate);
            heuristic_detail::validate_score(score);
            if (heuristic_detail::better(score, chosen_score, options.goal)) {
                chosen = candidate;
                chosen_score = score;
                if (options.mode == LocalSearchMode::first_improvement) break;
            }
        }
        ++result.iterations;
        if (!chosen) { result.termination = SearchTermination::local_optimum; break; }
        result.current_state = std::move(*chosen);
        result.best_state = result.current_state;
        result.current_score = result.best_score = chosen_score;
        ++result.accepted;
    }
    return result;
}
} // namespace cp
