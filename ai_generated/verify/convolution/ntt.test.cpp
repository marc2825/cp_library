#define PROBLEM "https://judge.yosupo.jp/problem/convolution_mod"
#include <iostream>
#include <vector>
#include "convolution/ntt.hpp"
int main() {
    int n, m; std::cin >> n >> m;
    std::vector<int> a(n), b(m);
    for (auto& x : a) std::cin >> x;
    for (auto& x : b) std::cin >> x;
    const auto size = n + m - 1;
    int padded = 1; while (padded < size) padded *= 2;
    a.resize(padded); b.resize(padded);
    cp::ntt(a); cp::ntt(b);
    for (int i = 0; i < padded; ++i) a[i] = static_cast<int>(static_cast<long long>(a[i]) * b[i] % 998244353);
    cp::ntt(a, true); a.resize(size);
    const auto& c = a;
    for (auto x : c) std::cout << x << ' ';
    std::cout << '\n';
}
