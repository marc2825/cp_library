#define PROBLEM "https://atcoder.jp/contests/abc172/tasks/abc172_e"
#include <iostream>
#include "combinatorics/permutation_combination.hpp"
int main() {
    int n, m; std::cin >> n >> m; cp::FactorialTable table(m, 1000000007);
    long long sum = 0;
    // 一致させる位置の個数ごとに包除する。全ての部分集合は列挙しない。
    for (int k = 0; k <= n; ++k) {
        const auto term = cp::mod_multiply(cp::combinations(table, n, k), cp::permutations(table, m - k, n - k), table.modulus());
        sum = (sum + (k % 2 ? -term : term) + table.modulus()) % table.modulus();
    }
    std::cout << cp::mod_multiply(cp::permutations(table, m, n), sum, table.modulus()) << '\n';
}
