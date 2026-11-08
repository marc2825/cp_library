#include <iostream>
#include "dp_optimization/optimal_solution_count.hpp"
int main() {
    auto lis = cp::lis_count(std::vector<int>{1,2,2,3});
    auto lcs = cp::lcs_alignment_count(std::string("aa"), std::string("a"));
    auto knapsack = cp::zero_one_knapsack_count(2, {{1,1},{1,1},{2,2}}).best();
    std::cout << lis.length << ' ' << lis.count << '\n'
              << lcs.length << ' ' << lcs.count << '\n'
              << knapsack.first << ' ' << knapsack.second << '\n';
}
