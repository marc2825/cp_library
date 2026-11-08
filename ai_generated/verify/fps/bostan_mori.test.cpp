#define PROBLEM "https://judge.yosupo.jp/problem/kth_term_of_linearly_recurrent_sequence"
#include <cstdint>
#include <iostream>
#include <vector>
#include "fps/bostan_mori.hpp"

int main() {
    std::ios::sync_with_stdio(false); std::cin.tie(nullptr);
    int n; std::uint64_t k; std::cin >> n >> k;
    std::vector<int> a(n); for (auto& x : a) std::cin >> x;
    std::vector<int> c(n); for (auto& x : c) std::cin >> x;
    std::cout << cp::linear_recurrence_nth(a, c, k) << '\n';
}
