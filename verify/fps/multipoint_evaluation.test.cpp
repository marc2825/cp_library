#define PROBLEM "https://judge.yosupo.jp/problem/multipoint_evaluation"
#include <cstdint>
#include <iostream>
#include <vector>
#include "fps/multipoint_evaluation.hpp"

void print(const std::vector<int>& a) {
    for (std::size_t i = 0; i < a.size(); ++i) std::cout << (i ? " " : "") << a[i];
    std::cout << '\n';
}
int main() {
    std::ios::sync_with_stdio(false); std::cin.tie(nullptr);
    int n, m; std::cin >> n >> m;
    std::vector<int> a(n); for (auto& x : a) std::cin >> x;
    std::vector<int> xs(m); for (auto& x : xs) std::cin >> x;
    auto result = cp::multipoint_evaluation(a, xs);
    print(result);
}
