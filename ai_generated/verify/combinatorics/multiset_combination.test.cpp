#define PROBLEM "https://cses.fi/problemset/task/1716"
#include <iostream>
#include "combinatorics/multiset_combination.hpp"
int main() {
    int n, k; std::cin >> n >> k; cp::FactorialTable table(n + k - 1, 1000000007);
    std::cout << cp::multiset_combination(table, n, k) << '\n';
}
