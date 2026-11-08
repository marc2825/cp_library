#define PROBLEM "https://judge.yosupo.jp/problem/convolution_mod"
#include <iostream>
#include <vector>
#include "convolution/convolution.hpp"
int main() {
    int n, m; std::cin >> n >> m;
    std::vector<int> a(n), b(m);
    for (auto& x : a) std::cin >> x;
    for (auto& x : b) std::cin >> x;
    const auto c = cp::convolution(a, b);
    for (auto x : c) std::cout << x << ' ';
    std::cout << '\n';
}
