#pragma once
#include <cstddef>
#include <deque>
#include <functional>
#include <limits>
#include <optional>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include "search_common.hpp"

namespace cp {
struct TabuSearchOptions : SearchOptions { std::size_t tenure = 7; };
// 訪問状態のkeyをtenure走査の間tabuにする。全体最良を更新する手はaspirationで許可。
// 悪化手も採用。docs/heuristic_search/tabu_search.md
template <class State, class Evaluate, class Neighbors, class KeyOf,
          class Key = std::decay_t<std::invoke_result_t<KeyOf&, const State&>>,
          class Hash = std::hash<Key>, class Equal = std::equal_to<Key>>
auto tabu_search(State initial, Evaluate evaluate, Neighbors neighbors, KeyOf key_of,
                 const TabuSearchOptions& options = {}, Hash hash = {}, Equal equal = {})
    -> SearchResult<State, heuristic_detail::ScoreOf<Evaluate, State>> {
    using Score = heuristic_detail::ScoreOf<Evaluate, State>;
    heuristic_detail::validate_goal(options.goal);
    const Score initial_score = evaluate(std::as_const(initial)); heuristic_detail::validate_score(initial_score);
    SearchResult<State, Score> result{initial, initial, initial_score, initial_score};
    std::unordered_map<Key, std::size_t, Hash, Equal> until(0, hash, equal);
    std::deque<std::pair<Key, std::size_t>> expiry;
    const auto remember = [&](const State& state, std::size_t end) {
        if (!options.tenure) return;
        Key key = key_of(state); until.insert_or_assign(key, end); expiry.emplace_back(std::move(key), end);
    };
    remember(initial, options.tenure);
    for (std::size_t step = 0; step < options.max_iterations; ++step) {
        if (options.stop && options.stop()) { result.termination = SearchTermination::stopped; return result; }
        while (!expiry.empty() && expiry.front().second <= step) {
            auto it = until.find(expiry.front().first);
            if (it != until.end() && it->second == expiry.front().second) until.erase(it);
            expiry.pop_front();
        }
        auto candidates = neighbors(std::as_const(result.current_state));
        std::optional<State> chosen; Score chosen_score = result.current_score;
        for (const auto& candidate : candidates) {
            if (options.stop && options.stop()) { result.termination = SearchTermination::stopped; return result; }
            heuristic_detail::count_evaluation(result.evaluations);
            const Score score = evaluate(candidate); heuristic_detail::validate_score(score);
            const auto found = until.find(key_of(candidate));
            if (found != until.end() && found->second > step && !heuristic_detail::better(score, result.best_score, options.goal)) continue;
            if (!chosen || heuristic_detail::better(score, chosen_score, options.goal)) { chosen = candidate; chosen_score = score; }
        }
        ++result.iterations;
        if (!chosen) { result.termination = SearchTermination::exhausted; return result; }
        result.current_state = std::move(*chosen); result.current_score = chosen_score; ++result.accepted;
        if (heuristic_detail::better(chosen_score, result.best_score, options.goal)) { result.best_state = result.current_state; result.best_score = chosen_score; }
        const auto max = std::numeric_limits<std::size_t>::max();
        const auto end = options.tenure >= max - step ? max : step + options.tenure + 1;
        remember(result.current_state, end);
    }
    return result;
}
} // namespace cp
