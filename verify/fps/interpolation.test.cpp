#define PROBLEM "https://judge.yosupo.jp/problem/polynomial_interpolation"
#include <cstdint>
#include <iostream>
#include <vector>
#include "fps/interpolation.hpp"

void print(const std::vector<int>& a) {
    for (std::size_t i = 0; i < a.size(); ++i) std::cout << (i ? " " : "") << a[i];
    std::cout << '\n';
}
int main() {
    std::ios::sync_with_stdio(false); std::cin.tie(nullptr);
    int n; std::cin >> n;
    std::vector<int> xs(n), ys(n);
    for (auto& x : xs) std::cin >> x;
    for (auto& y : ys) std::cin >> y;
    auto result = cp::polynomial_interpolation(xs, ys);
    print(result);
}
