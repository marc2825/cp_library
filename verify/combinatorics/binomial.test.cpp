#define PROBLEM "https://cses.fi/problemset/task/1079"
#include <algorithm>
#include <iostream>
#include <utility>
#include <vector>
#include "combinatorics/binomial.hpp"
int main() {
    std::ios::sync_with_stdio(false); std::cin.tie(nullptr);
    int q, bound = 0; std::cin >> q; std::vector<std::pair<int, int>> queries(q);
    for (auto& [n, k] : queries) { std::cin >> n >> k; bound = std::max(bound, n); }
    cp::FactorialTable table(bound, 1000000007);
    for (auto [n, k] : queries) std::cout << cp::binomial(table, n, k) << '\n';
}
