#include <iostream>
#include "dp_optimization/branch_bound_knapsack.hpp"
int main() {
    auto r=cp::branch_bound_knapsack(7,{{3,4},{4,5},{5,9}});
    std::cout << r.value << '\n';
}
