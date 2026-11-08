#define PROBLEM "https://judge.yosupo.jp/problem/pow_of_matrix"
#include <cstdint>
#include <iostream>
#include <vector>
#include "linear_recurrence/matrix_recurrence.hpp"
int main() {
    std::ios::sync_with_stdio(false); std::cin.tie(nullptr);
    int d; std::uint64_t n; std::cin >> d >> n;
    std::vector<std::vector<int>> a(d, std::vector<int>(d));
    for (auto& row : a) for (auto& x : row) std::cin >> x;
    for (const auto& row : cp::matrix_power_mod(a, n)) {
        for (std::size_t j = 0; j < row.size(); ++j) std::cout << (j ? " " : "") << row[j];
        std::cout << '\n';
    }
}
