#include <iostream>
#include <vector>
#include "heuristic_search/local_search.hpp"
int main() {
    cp::LocalSearchOptions options;
    options.mode = cp::LocalSearchMode::best_improvement;
    const auto result = cp::local_search(8,
        [](int x) { return (x - 3) * (x - 3); },
        [](int x) { return std::vector<int>{x - 1, x + 1}; }, options);
    std::cout << result.best_state << ' ' << result.best_score << '\n';
}
