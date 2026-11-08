#include "fps/composed_polynomial.hpp"
#include <iostream>

template <class T> void print(const std::vector<T> &a) {
    for (std::size_t i = 0; i < a.size(); ++i)
        std::cout << (i ? " " : "") << a[i];
    std::cout << '\n';
}
int main() {
    print(cp::polynomial_composed_sum({2, -3, 1}, {-3, 1}));
    print(cp::polynomial_composed_product({2, -3, 1}, {-3, 1}));
}
