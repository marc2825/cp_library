#include "fps/polynomial_resultant.hpp"
#include <iostream>

template <class T> void print(const std::vector<T> &a) {
    for (std::size_t i = 0; i < a.size(); ++i)
        std::cout << (i ? " " : "") << a[i];
    std::cout << '\n';
}
int main() {
    std::cout << cp::polynomial_resultant({2, -3, 1}, {-3, 1}) << "\n";
    std::cout << cp::polynomial_discriminant({2, -3, 1}) << "\n";
}
