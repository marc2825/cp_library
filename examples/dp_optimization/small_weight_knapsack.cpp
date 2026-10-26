#include <iostream>
#include "dp_optimization/small_weight_knapsack.hpp"
int main() {
    auto result = cp::small_weight_knapsack(7, {{2,5},{2,4},{2,1},{3,7},{3,6},{0,2}});
    std::cout << result.best_value() << '\n';
}
