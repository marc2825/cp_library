#pragma once
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <utility>
#include "random.hpp"
#include "search_common.hpp"

namespace cp {
enum class CoolingSchedule { geometric, linear };
struct AnnealingOptions : SearchOptions {
    long double start_temperature = 1, end_temperature = 0.01L;
    CoolingSchedule cooling = CoolingSchedule::geometric;
    std::function<long double()> progress; // Optional time-based schedule, in [0, 1].
};
inline long double annealing_temperature(const AnnealingOptions& options, long double progress) {
    if (!std::isfinite(options.start_temperature) || options.start_temperature <= 0 ||
        !std::isfinite(options.end_temperature) || options.end_temperature <= 0 ||
        !std::isfinite(progress) || progress < 0 || progress > 1 ||
        (options.cooling != CoolingSchedule::linear && options.cooling != CoolingSchedule::geometric))
        throw std::invalid_argument("invalid annealing schedule");
    if (progress == 0) return options.start_temperature;
    if (progress == 1) return options.end_temperature;
    const long double value = options.cooling == CoolingSchedule::linear
        ? (1 - progress) * options.start_temperature + progress * options.end_temperature
        : std::exp((1 - progress) * std::log(options.start_temperature) + progress * std::log(options.end_temperature));
    return std::clamp(value, std::min(options.start_temperature, options.end_temperature),
                      std::max(options.start_temperature, options.end_temperature));
}
// propose(const State&, Random&) returns a candidate by value; evaluate is pure.
// State copies must be independent; callbacks must not mutate their input.
template<class State, class Evaluate, class Propose>
auto simulated_annealing(State initial, Evaluate evaluate, Propose propose, Random& random,
                         const AnnealingOptions& options = {})
    -> SearchResult<State, heuristic_detail::ScoreOf<Evaluate, State>> {
    using Score = heuristic_detail::ScoreOf<Evaluate, State>;
    heuristic_detail::validate_goal(options.goal);
    (void)annealing_temperature(options, 0);
    const Score initial_score = evaluate(std::as_const(initial));
    heuristic_detail::validate_score(initial_score);
    SearchResult<State, Score> result{initial, initial, initial_score, initial_score};
    for (std::size_t step = 0; step < options.max_iterations; ++step) {
        if (options.stop && options.stop()) { result.termination = SearchTermination::stopped; break; }
        State candidate = propose(std::as_const(result.current_state), random);
        heuristic_detail::count_evaluation(result.evaluations);
        const Score score = evaluate(std::as_const(candidate));
        heuristic_detail::validate_score(score);
        const long double progress = options.progress ? options.progress() : (options.max_iterations <= 1 ? 0 :
            static_cast<long double>(step) / static_cast<long double>(options.max_iterations - 1));
        const long double temperature = annealing_temperature(options, progress);
        bool accept = !heuristic_detail::better(result.current_score, score, options.goal);
        if (!accept) {
            const auto loss = heuristic_detail::worsening_loss(result.current_score, score, options.goal);
            accept = random.real() < std::exp(-loss / temperature);
        }
        if (accept) {
            result.current_state = std::move(candidate);
            result.current_score = score;
            ++result.accepted;
            if (heuristic_detail::better(score, result.best_score, options.goal)) {
                result.best_state = result.current_state;
                result.best_score = score;
            }
        }
        ++result.iterations;
    }
    return result;
}
} // namespace cp
