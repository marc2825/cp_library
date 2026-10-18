#include <iostream>
#include <vector>
#include "../../heuristic_search/parameter_tuning.hpp"

int main() {
    auto result = cp::tune_parameters(std::vector<int>{3, 1, 2}, std::vector<std::uint64_t>{1, 2}, [](int setting, std::uint64_t seed) { return setting * setting + int(seed); });
    std::cout << result.best_index << "\n";
}
