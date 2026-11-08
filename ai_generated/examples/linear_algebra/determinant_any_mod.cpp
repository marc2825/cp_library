#include "linear_algebra/determinant_any_mod.hpp"
#include <iostream>

template <class T> void print(const std::vector<T> &a) {
    for (std::size_t i = 0; i < a.size(); ++i)
        std::cout << (i ? " " : "") << a[i];
    std::cout << '\n';
}

int main() {
    cp::Matrix<int> a{{2, 4}, {6, 8}};
    std::cout << cp::determinant_any_mod(a, 12) << "\n";
}
