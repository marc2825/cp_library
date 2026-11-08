#include <iostream>
#include <vector>
#include "../../heuristic_search/multi_start_sa.hpp"

int main() {
    cp::AnnealingOptions options; options.max_iterations = 0;
    auto result = cp::multi_start_sa(3, [](std::size_t i, cp::Random&) { return int(i); }, [](int x) { return x; }, [](int x, cp::Random&) { return x; }, 1, options);
    std::cout << result.runs.size() << " " << result.best_run().best_score << "\n";
}
