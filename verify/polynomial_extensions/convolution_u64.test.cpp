#define PROBLEM "https://judge.yosupo.jp/problem/convolution_mod_2_64"
#include "convolution/convolution_u64.hpp"
#include <iostream>
int main() {
    std::ios::sync_with_stdio(false);
    std::cin.tie(nullptr);
    std::size_t n, m;
    if (!(std::cin >> n >> m))
        return 0;
    std::vector<std::uint64_t> a(n), b(m);
    for (auto &x : a)
        std::cin >> x;
    for (auto &x : b)
        std::cin >> x;
    auto c = cp::convolution_u64(a, b);
    for (std::size_t i = 0; i < c.size(); ++i)
        std::cout << (i ? " " : "") << c[i];
    std::cout << '\n';
}
