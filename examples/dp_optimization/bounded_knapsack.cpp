#include <iostream>
#include "dp_optimization/bounded_knapsack.hpp"
int main() {
    auto result = cp::bounded_knapsack(7, {{3,4,1}, {2,3,2}}); // {重さ,価値,個数}
    std::cout << result.best_value() << '\n';
}
