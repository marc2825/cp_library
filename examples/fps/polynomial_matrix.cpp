#include "fps/polynomial_matrix.hpp"
#include <iostream>

template <class T> void print(const std::vector<T> &a) {
    for (std::size_t i = 0; i < a.size(); ++i)
        std::cout << (i ? " " : "") << a[i];
    std::cout << '\n';
}
int main() {
    cp::PolynomialMatrix a{{{0, 1}, {1}}, {{1}, {0, 1}}};
    print(cp::polynomial_matrix_determinant(a));
    auto p = cp::polynomial_matrix_prefix_product(a, 0, 3);
    for (const auto &row : p)
        print(row);
}
