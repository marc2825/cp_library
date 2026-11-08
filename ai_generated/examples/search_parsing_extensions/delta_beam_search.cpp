#include "heuristic_search/delta_beam_search.hpp"
#include <iostream>
using namespace cp;
int main() {
    int root = 0;
    auto score = [](const int &x) { return (x - 5) * (x - 5); };
    BeamSearchOptions options;
    options.width = 3;
    options.max_depth = 4;
    auto result = delta_beam_search(
        root, score, [](const int &) { return std::vector<int>{-1, 2}; },
        [&](const int &x, const int &m) { return score(x + m) - score(x); },
        [](int &x, const int &m) { x += m; }, [](int &x, const int &m) noexcept { x -= m; }, options);
    std::cout << result.best_state << " " << result.best_score << " " << root << "\n";
}
