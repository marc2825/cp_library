#define PROBLEM "https://judge.yosupo.jp/problem/find_linear_recurrence"
#include <iostream>
#include <vector>
#include "linear_recurrence/berlekamp_massey.hpp"
int main() {
    std::ios::sync_with_stdio(false); std::cin.tie(nullptr);
    int n; std::cin >> n;
    std::vector<int> s(n); for (auto& x : s) std::cin >> x;
    const auto c = cp::berlekamp_massey(s);
    std::cout << c.size() << '\n';
    for (std::size_t i = 0; i < c.size(); ++i) std::cout << (i ? " " : "") << c[i];
    std::cout << '\n';
}
