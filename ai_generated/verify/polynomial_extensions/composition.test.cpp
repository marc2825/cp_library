#define PROBLEM "https://judge.yosupo.jp/problem/composition_of_formal_power_series"
#include "fps/fast_composition.hpp"
#include <iostream>
int main() {
    std::ios::sync_with_stdio(false);
    std::cin.tie(nullptr);
    std::size_t n;
    if (!(std::cin >> n))
        return 0;
    std::vector<int> a(n), b(n);
    for (auto &x : a)
        std::cin >> x;
    for (auto &x : b)
        std::cin >> x;
    auto c = cp::fps_compose_fast(a, b, n);
    for (std::size_t i = 0; i < c.size(); ++i)
        std::cout << (i ? " " : "") << c[i];
    std::cout << '\n';
}
