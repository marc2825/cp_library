#define PROBLEM "https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=DSL_3_D"
#include "static_query/sliding_window_minimum.hpp"
#include <iostream>
#include <vector>
int main() {
    std::ios::sync_with_stdio(false); std::cin.tie(nullptr);
    int n, k; std::cin >> n >> k;
    std::vector<int> a(n); for (auto& x : a) std::cin >> x;
    auto result = cp::sliding_window_minimum(a, k);
    for (int i = 0; i < int(result.size()); ++i) std::cout << (i ? " " : "") << result[i];
    std::cout << '\n';
}
