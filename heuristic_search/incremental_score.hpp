#pragma once
#include "../number_theory_convolution/detail/arithmetic.hpp"
#include "simulated_annealing.hpp"
#include <cmath>
#include <type_traits>
#include <utility>

namespace cp {
// delta(const State&,move)はScore型。apply後、拒否またはacceptの例外でundo。
// applyは強い例外保証、undoは例外を投げないこと。docs/heuristic_search/incremental_score.md
template <class State, class Score, class Move, class Delta, class Apply, class Undo, class Accept>
bool try_scored_move(State &state, Score &score, const Move &move, Delta delta, Apply apply, Undo undo,
                     Accept accept) {
    static_assert(std::is_signed_v<Score>, "incremental scores must be signed");
    static_assert(
        std::is_same_v<std::decay_t<std::invoke_result_t<Delta &, const State &, const Move &>>, Score>,
        "delta must return the same Score type");
    heuristic_detail::validate_score(score);
    const Score old = score;
    const Score difference = delta(std::as_const(state), move);
    heuristic_detail::validate_score(difference);
    Score candidate = old;
    detail::arithmetic_add_assign(candidate, difference);
    heuristic_detail::validate_score(candidate);
    apply(state, move);
    score = candidate;
    bool accepted;
    try {
        accepted = accept(std::as_const(state), old, candidate);
    } catch (...) {
        score = old;
        undo(state, move);
        throw;
    }
    if (!accepted) {
        score = old;
        undo(state, move);
    }
    return accepted;
}
// 初期評価を1回だけ行い、以降はMoveの差分評価とin-place apply/undoでSA。
// evaluationsは初期評価+差分評価の回数。最良解を更新したときだけStateを保存。
template <class State, class Evaluate, class Propose, class Delta, class Apply, class Undo, class AcceptWorse>
auto simulated_annealing_delta_acceptance(State initial, Evaluate evaluate, Propose propose, Delta delta,
                                          Apply apply, Undo undo, Random &random, AcceptWorse accept_worse,
                                          const AnnealingOptions &options = {})
    -> SearchResult<State, heuristic_detail::ScoreOf<Evaluate, State>> {
    using Score = heuristic_detail::ScoreOf<Evaluate, State>;
    heuristic_detail::validate_goal(options.goal);
    (void)annealing_temperature(options, 0);
    const Score initial_score = evaluate(std::as_const(initial));
    heuristic_detail::validate_score(initial_score);
    SearchResult<State, Score> result{initial, std::move(initial), initial_score, initial_score};
    for (std::size_t step = 0; step < options.max_iterations; ++step) {
        if (options.stop && options.stop()) {
            result.termination = SearchTermination::stopped;
            break;
        }
        const auto move = propose(std::as_const(result.current_state), random);
        heuristic_detail::count_evaluation(result.evaluations);
        const bool accepted = try_scored_move(
            result.current_state, result.current_score, move, delta, apply, undo,
            [&](const State &, Score old, Score score) {
                const long double progress =
                    options.progress ? options.progress()
                                     : (options.max_iterations <= 1
                                            ? 0
                                            : static_cast<long double>(step) / (options.max_iterations - 1));
                const auto temperature = annealing_temperature(options, progress);
                if (!heuristic_detail::better(old, score, options.goal))
                    return true;
                return accept_worse(heuristic_detail::worsening_loss(old, score, options.goal), temperature,
                                    random);
            });
        if (accepted) {
            ++result.accepted;
            if (heuristic_detail::better(result.current_score, result.best_score, options.goal)) {
                result.best_state = result.current_state;
                result.best_score = result.current_score;
            }
        }
        ++result.iterations;
    }
    return result;
}
template <class State, class Evaluate, class Propose, class Delta, class Apply, class Undo>
auto simulated_annealing_delta(State initial, Evaluate evaluate, Propose propose, Delta delta, Apply apply,
                               Undo undo, Random &random, const AnnealingOptions &options = {}) {
    return simulated_annealing_delta_acceptance(
        std::move(initial), evaluate, propose, delta, apply, undo, random,
        [](long double loss, long double temperature, Random &rng) {
            return rng.real() < std::exp(-loss / temperature);
        },
        options);
}
} // namespace cp
