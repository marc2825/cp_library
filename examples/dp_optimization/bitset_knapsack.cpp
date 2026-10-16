#include <iostream>
#include "dp_optimization/bitset_knapsack.hpp"
int main() {
    cp::BitsetKnapsack sums(12);
    for (int weight : {3,5,6}) sums.add(weight);
    for (int sum : sums.reachable_sums()) std::cout << sum << ' ';
    std::cout << '\n';
    std::cout << sums.contains(11) << '\n';
}
