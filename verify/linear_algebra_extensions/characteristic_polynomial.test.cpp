#define PROBLEM "https://judge.yosupo.jp/problem/characteristic_polynomial"
#include "linear_algebra/characteristic_polynomial_fast.hpp"
#include <iostream>
int main() {
    std::ios::sync_with_stdio(false);
    std::cin.tie(nullptr);
    std::size_t n;
    std::cin >> n;
    cp::Matrix<int> a(n, n);
    for (std::size_t i = 0; i < n; ++i)
        for (std::size_t j = 0; j < n; ++j)
            std::cin >> a(i, j);
    auto result = cp::characteristic_polynomial_fast(a);
    for (std::size_t i = 0; i < result.coefficients.size(); ++i)
        std::cout << (i ? " " : "") << result.coefficients[i];
    std::cout << '\n';
}
