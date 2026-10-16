#define PROBLEM "https://judge.yosupo.jp/problem/polynomial_taylor_shift"
#include <cstdint>
#include <iostream>
#include <vector>
#include "fps/taylor_shift.hpp"

void print(const std::vector<int>& a) {
    for (std::size_t i = 0; i < a.size(); ++i) std::cout << (i ? " " : "") << a[i];
    std::cout << '\n';
}
int main() {
    std::ios::sync_with_stdio(false); std::cin.tie(nullptr);
    int n; long long c; std::cin >> n >> c;
    std::vector<int> a(n); for (auto& x : a) std::cin >> x;
    auto result = cp::taylor_shift(a, c);
    print(result);
}
