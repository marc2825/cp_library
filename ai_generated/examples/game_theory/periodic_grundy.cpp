#include <iostream>
#include "game_theory/periodic_grundy.hpp"
int main() {
    const auto candidate = cp::detect_grundy_period({9, 0, 1, 0, 1, 0, 1});
    std::cout << candidate->preperiod << ' ' << candidate->period << '\n';
    const auto g = cp::subtraction_grundy({1, 3, 4}, 40);
    std::cout << g.period->preperiod << ' ' << g.period->period << '\n';
    std::cout << g.value(UINT64_MAX) << '\n';
}
