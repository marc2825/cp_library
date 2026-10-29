#include "heuristic_search/chokudai_search.hpp"
#include <iostream>
using namespace cp;
int main() {
    ChokudaiOptions options;
    options.max_depth = 4;
    options.max_expansions = 100;
    options.max_nodes = 1000;
    auto result = chokudai_search(
        0, [](int x) { return (x - 5) * (x - 5); }, [](int x) { return std::vector<int>{x - 1, x + 2}; },
        options);
    std::cout << result.best().state << " " << result.best().score << " " << result.reached_target() << "\n";
}
