#include <iostream>
#include <vector>
#include "fps/polynomial.hpp"

void print(const std::vector<int>& a) {
    for (int x : a) std::cout << x << ' ';
    std::cout << '\n';
}
int main() {
    const std::vector<int> a{1, 2, 3}, b{-1, 1};
    print(cp::polynomial_add<17>(a, b));
    print(cp::polynomial_subtract<17>(a, b));
    print(cp::polynomial_multiply<17, 3>(a, b));
    print(cp::polynomial_derivative<17>(a));
    print(cp::polynomial_integral<17>(a));
    print(cp::fps_truncate<17>(a, 5));
}
