#define PROBLEM "https://judge.yosupo.jp/problem/sqrt_of_formal_power_series"
#include <cstdint>
#include <iostream>
#include <vector>
#include "fps/sqrt.hpp"

void print(const std::vector<int>& a) {
    for (std::size_t i = 0; i < a.size(); ++i) std::cout << (i ? " " : "") << a[i];
    std::cout << '\n';
}
int main() {
    std::ios::sync_with_stdio(false); std::cin.tie(nullptr);
    int n; std::cin >> n;
    std::vector<int> a(n); for (auto& x : a) std::cin >> x;
    auto result = cp::fps_sqrt(a, a.size());
    if (result) print(*result); else std::cout << -1 << '\n';
}
