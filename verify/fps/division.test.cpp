#define PROBLEM "https://judge.yosupo.jp/problem/division_of_polynomials"
#include <cstdint>
#include <iostream>
#include <vector>
#include "fps/division.hpp"

void print(const std::vector<int>& a) {
    for (std::size_t i = 0; i < a.size(); ++i) std::cout << (i ? " " : "") << a[i];
    std::cout << '\n';
}
int main() {
    std::ios::sync_with_stdio(false); std::cin.tie(nullptr);
    int n, m; std::cin >> n >> m;
    std::vector<int> a(n); for (auto& x : a) std::cin >> x;
    std::vector<int> b(m); for (auto& x : b) std::cin >> x;
    const auto [q, r] = cp::polynomial_divmod(a, b);
    std::cout << q.size() << ' ' << r.size() << '\n';
    print(q); print(r);
}
