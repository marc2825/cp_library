#include <iostream>
#include "heuristic_search/simulated_annealing.hpp"
int main() {
    cp::Random random(42);
    cp::AnnealingOptions options;
    options.max_iterations = 12;
    options.start_temperature = 10;
    options.end_temperature = 0.1;
    const auto result = cp::simulated_annealing(8,
        [](int x) { return x * x; },
        [](int x, cp::Random&) { return x > 0 ? x - 1 : x; }, random, options);
    std::cout << result.best_state << ' ' << result.best_score << '\n';
    // Time budget: options.stop = [&] { return timer.expired(); };
    // options.progress = [&] { return timer.progress(); };
}
