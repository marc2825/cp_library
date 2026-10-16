#define PROBLEM "https://cses.fi/problemset/task/1654"
#include <algorithm>
#include <iostream>
#include <vector>
#include "set_function/sos_dp.hpp"
#include "set_function/superset_zeta_transform.hpp"
int main() {
    std::ios::sync_with_stdio(false); std::cin.tie(nullptr);
    int n; std::cin >> n; std::vector<int> a(n); int maximum = 0;
    for (auto& x : a) { std::cin >> x; maximum = std::max(maximum, x); }
    std::size_t size = 1; while (size <= static_cast<std::size_t>(maximum)) size *= 2;
    std::vector<int> lower(size), upper;
    for (int x : a) ++lower[x];
    upper = lower; cp::sos_dp(lower); cp::superset_zeta_transform(upper);
    for (int x : a) std::cout << lower[x] << ' ' << upper[x] << ' ' << n - lower[(size - 1) ^ x] << '\n';
}
