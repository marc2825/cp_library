#include <iostream>
#include "dp_optimization/unbounded_knapsack.hpp"
int main() {
    auto result = cp::unbounded_knapsack(7, {{3,4}, {4,5}, {2,3}});
    std::cout << result.best_value() << '\n';
}
