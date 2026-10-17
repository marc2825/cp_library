#include <iostream>
#include <vector>
#include "heuristic_search/tabu_search.hpp"
int main() {
    cp::TabuSearchOptions options; options.tenure = 1; options.max_iterations = 3;
    auto r = cp::tabu_search(0, [](int x) { return x; },
        [](int x) { return std::vector<int>{1 - x}; }, [](int x) { return x; }, options);
    std::cout << r.best_state << ' ' << r.current_state << ' ' << r.accepted << '\n';
}
