#include <iostream>
#include "heuristic_search/incremental_score.hpp"
int main() {
    long long state = 4, score = 16;
    auto delta = [](const long long& x, const long long& d) -> long long { return 2 * x * d + d * d; };
    auto apply = [](long long& x, long long d) { x += d; };
    auto undo = [](long long& x, long long d) { x -= d; };
    auto accept = [](const long long&, long long old, long long next) { return next < old; };
    std::cout << cp::try_scored_move(state, score, -1LL, delta, apply, undo, accept) << ' ' << state << ' ' << score << '\n';
    std::cout << cp::try_scored_move(state, score, 2LL, delta, apply, undo, accept) << ' ' << state << ' ' << score << '\n';
    cp::Random random(0); cp::AnnealingOptions options; options.max_iterations = 4;
    auto r = cp::simulated_annealing_delta(4LL, [](const long long& x) { return x * x; },
        [](const long long&, cp::Random&) { return -1LL; }, delta, apply, undo, random, options);
    std::cout << r.current_state << ' ' << r.current_score << ' ' << r.evaluations << '\n';
}
