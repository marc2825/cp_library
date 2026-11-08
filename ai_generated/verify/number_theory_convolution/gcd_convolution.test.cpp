#define PROBLEM "https://judge.yosupo.jp/problem/gcd_convolution"
#include <iostream>
#include <vector>
#include "number_theory_convolution/gcd_convolution.hpp"
#include "modint.hpp"
int main() {
    std::ios::sync_with_stdio(false); std::cin.tie(nullptr);
    int n; std::cin >> n;
    std::vector<ModInt> a(n + 1), b(n + 1);
    for (int i = 1, x; i <= n; ++i) { std::cin >> x; a[i] = ModInt(x); }
    for (int i = 1, x; i <= n; ++i) { std::cin >> x; b[i] = ModInt(x); }
    const auto c = cp::gcd_convolution(a, b);
    for (int i = 1; i <= n; ++i) std::cout << c[i].value << (i == n ? '\n' : ' ');
}
