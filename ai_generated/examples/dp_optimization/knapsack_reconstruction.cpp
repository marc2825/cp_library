#include <iostream>
#include "dp_optimization/knapsack_reconstruction.hpp"
int main() {
    auto result = cp::zero_one_knapsack_reconstruct(5, {{0,3},{2,4},{3,5},{2,4}});
    std::cout << result.values.best_value();
    auto counts = result.restore(result.best_weight());
    for (auto count : *counts) std::cout << ' ' << count;
    std::cout << '\n';
    cp::BitsetSubsetSumReconstruction subset(5);
    subset.add(2); subset.add(3); subset.add(2);
    auto ids = subset.restore(5);
    for (std::size_t i = 0; i < ids->size(); ++i) std::cout << (i ? " " : "") << (*ids)[i];
    std::cout << '\n';
}
