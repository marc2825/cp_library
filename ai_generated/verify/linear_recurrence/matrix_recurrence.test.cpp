#define PROBLEM "https://judge.yosupo.jp/problem/kth_term_of_linearly_recurrent_sequence"
#include <cstdint>
#include <iostream>
#include <vector>
#include "linear_recurrence/matrix_recurrence.hpp"
int main() {
    std::ios::sync_with_stdio(false); std::cin.tie(nullptr);
    int d; std::uint64_t n; std::cin >> d >> n;
    std::vector<int> a(d), c(d);
    for (auto& x : a) std::cin >> x;
    for (auto& x : c) std::cin >> x;
    std::cout << cp::linear_recurrence_matrix(a, c, n) << '\n';
}
