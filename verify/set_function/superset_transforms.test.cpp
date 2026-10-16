#define PROBLEM "https://judge.yosupo.jp/problem/bitwise_and_convolution"
#include <algorithm>
#include <iostream>
#include <vector>
#include "set_function/superset_zeta_transform.hpp"
#include "set_function/superset_mobius_transform.hpp"
#include "../number_theory_convolution/modint.hpp"
int main() {
    std::ios::sync_with_stdio(false); std::cin.tie(nullptr);
    int bits; std::cin >> bits;
    std::vector<ModInt> a(std::size_t(1) << bits), b(a.size());
    for (auto& x : a) { int v; std::cin >> v; x = ModInt(v); }
    for (auto& x : b) { int v; std::cin >> v; x = ModInt(v); }
    cp::superset_zeta_transform(a); cp::superset_zeta_transform(b);
    for (std::size_t i = 0; i < a.size(); ++i) a[i] = a[i] * b[i];
    cp::superset_mobius_transform(a);
    for (std::size_t i = 0; i < a.size(); ++i) std::cout << (i ? " " : "") << a[i].value;
    std::cout << '\n';
}
