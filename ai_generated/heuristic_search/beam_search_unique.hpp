#pragma once
#include <algorithm>
#include <functional>
#include <type_traits>
#include <unordered_set>
#include <utility>
#include <vector>
#include "beam_search.hpp"

namespace cp {
// 同じ深さの同一keyを統合し、最良score（同点なら先着）を残してから幅を制限。
// 既存beam_searchは重複を保持。docs/heuristic_search/beam_search_unique.md
template <class State, class Evaluate, class Expand, class KeyOf,
          class Key = std::decay_t<std::invoke_result_t<KeyOf&, const State&>>,
          class Hash = std::hash<Key>, class Equal = std::equal_to<Key>>
auto beam_search_unique(State initial, Evaluate evaluate, Expand expand, KeyOf key_of,
                        const BeamSearchOptions& options = {}, Hash hash = {}, Equal equal = {})
    -> BeamSearchResult<State, heuristic_detail::ScoreOf<Evaluate, State>> {
    using Score = heuristic_detail::ScoreOf<Evaluate, State>; using Entry = ScoredSearchState<State, Score>;
    heuristic_detail::validate_goal(options.goal);
    if (!options.width) throw std::invalid_argument("beam width must be positive");
    const Score initial_score = evaluate(std::as_const(initial)); heuristic_detail::validate_score(initial_score);
    BeamSearchResult<State, Score> result{{Entry{std::move(initial), initial_score}}};
    for (std::size_t depth = 0; depth < options.max_depth; ++depth) {
        std::vector<Entry> next;
        for (const auto& entry : result.frontier) {
            if (options.stop && options.stop()) { result.termination = SearchTermination::stopped; return result; }
            auto children = expand(entry.state);
            for (auto& child : children) {
                if (options.stop && options.stop()) { result.termination = SearchTermination::stopped; return result; }
                heuristic_detail::count_evaluation(result.evaluations);
                const Score score = evaluate(std::as_const(child)); heuristic_detail::validate_score(score);
                next.push_back({std::move(child), score});
            }
        }
        if (next.empty()) { result.termination = SearchTermination::exhausted; break; }
        std::stable_sort(next.begin(), next.end(), [&](const Entry& a, const Entry& b) { return heuristic_detail::better(a.score, b.score, options.goal); });
        std::unordered_set<Key, Hash, Equal> seen(0, hash, equal); std::vector<Entry> unique;
        for (auto& entry : next) if (seen.emplace(key_of(std::as_const(entry.state))).second) {
            unique.push_back(std::move(entry)); if (unique.size() == options.width) break;
        }
        result.frontier = std::move(unique); ++result.depth;
    }
    return result;
}
} // namespace cp
