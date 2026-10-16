#define PROBLEM "https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=DPL_1_B"
#include <iostream>
#include <vector>
#include "dp_optimization/zero_one_knapsack.hpp"
int main() {
    std::ios::sync_with_stdio(false); std::cin.tie(nullptr);
    int n,capacity;std::cin>>n>>capacity;std::vector<cp::KnapsackItem> items(n);
    for(auto& item:items)std::cin >> item.value >> item.weight;
    std::cout << cp::zero_one_knapsack(capacity,items).best_value() << '\n';
}
