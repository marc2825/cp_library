#include <iostream>
#include "heuristic_search/beam_search_unique.hpp"
int main() {
    cp::BeamSearchOptions options; options.width = 3; options.max_depth = 1;
    auto r = cp::beam_search_unique(0, [](int x) { return x; },
        [](int) { return std::vector<int>{1, 1, 2, 3}; }, [](int x) { return x; }, options);
    for (std::size_t i = 0; i < r.frontier.size(); ++i) {
        if (i) std::cout << ' ';
        std::cout << r.frontier[i].state;
    }
    std::cout << '\n';
}
