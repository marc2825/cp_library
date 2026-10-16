#include <iostream>
#include "dp_optimization/zero_one_knapsack.hpp"
int main() {
    auto result = cp::zero_one_knapsack(7, {{3,4}, {4,5}, {2,3}}); // {重さ,価値}
    std::cout << result.best_value() << '\n';
    if (result.dp[7]) std::cout << *result.dp[7] << '\n'; // 重さちょうど7。
}
