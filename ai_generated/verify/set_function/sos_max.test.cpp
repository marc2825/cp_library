#define PROBLEM "https://codeforces.com/problemset/problem/165/E"
#include <algorithm>
#include <iostream>
#include <vector>
#include "set_function/sos_dp.hpp"
int main() {
    std::ios::sync_with_stdio(false); std::cin.tie(nullptr);
    int n; std::cin >> n; std::vector<int> a(n); int maximum = 0;
    for (auto& x : a) { std::cin >> x; maximum = std::max(maximum, x); }
    std::size_t size = 1; while (size <= static_cast<std::size_t>(maximum)) size *= 2;
    std::vector<int> best(size, -1); for (int x : a) best[x] = x;
    cp::sos_dp(best, [](int x, int y) { return std::max(x, y); });
    for (int i = 0; i < n; ++i) std::cout << (i ? " " : "") << best[(size - 1) ^ a[i]];
    std::cout << '\n';
}
