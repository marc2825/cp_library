#pragma once
#include <algorithm>
#include <cstddef>
#include <functional>
#include <stdexcept>
#include <utility>
#include <vector>
#include "search_common.hpp"

namespace cp {
struct BeamSearchOptions {
    std::size_t width = 10, max_depth = 100;
    SearchGoal goal = SearchGoal::minimize;
    std::function<bool()> stop;
};
template<class State, class Score> struct ScoredSearchState { State state; Score score; };
template<class State, class Score> struct BeamSearchResult {
    std::vector<ScoredSearchState<State, Score>> frontier;
    std::size_t depth = 0, evaluations = 1;
    SearchTermination termination = SearchTermination::depth_limit;
    const ScoredSearchState<State, Score>& best() const { return frontier.front(); }
};
// expand(const State&) returns a range of children by value. Scores are compared
// within a depth; ties retain parent/child order. Duplicates are retained.
template<class State, class Evaluate, class Expand>
auto beam_search(State initial, Evaluate evaluate, Expand expand, const BeamSearchOptions& options = {})
    -> BeamSearchResult<State, heuristic_detail::ScoreOf<Evaluate, State>> {
    using Score = heuristic_detail::ScoreOf<Evaluate, State>;
    using Entry = ScoredSearchState<State, Score>;
    heuristic_detail::validate_goal(options.goal);
    if (!options.width) throw std::invalid_argument("beam width must be positive");
    const Score initial_score = evaluate(std::as_const(initial));
    heuristic_detail::validate_score(initial_score);
    BeamSearchResult<State, Score> result{{Entry{std::move(initial), initial_score}}};
    for (std::size_t depth = 0; depth < options.max_depth; ++depth) {
        std::vector<Entry> next;
        for (const auto& entry : result.frontier) {
            if (options.stop && options.stop()) { result.termination = SearchTermination::stopped; return result; }
            auto children = expand(entry.state);
            for (auto& child : children) {
                if (options.stop && options.stop()) { result.termination = SearchTermination::stopped; return result; }
                heuristic_detail::count_evaluation(result.evaluations);
                const Score score = evaluate(std::as_const(child));
                heuristic_detail::validate_score(score);
                next.push_back({std::move(child), score});
            }
        }
        if (next.empty()) { result.termination = SearchTermination::exhausted; break; }
        std::stable_sort(next.begin(), next.end(), [&](const Entry& a, const Entry& b) {
            return heuristic_detail::better(a.score, b.score, options.goal);
        });
        if (next.size() > options.width) next.erase(next.begin() + options.width, next.end());
        result.frontier = std::move(next);
        ++result.depth;
    }
    return result;
}
} // namespace cp
