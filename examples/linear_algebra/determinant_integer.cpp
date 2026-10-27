#include "linear_algebra/determinant_integer.hpp"
#include <iostream>

template <class T> void print(const std::vector<T> &a) {
    for (std::size_t i = 0; i < a.size(); ++i)
        std::cout << (i ? " " : "") << a[i];
    std::cout << '\n';
}

int main() {
    cp::BigInt x("100000000000000000001");
    cp::Matrix<cp::BigInt> a{{x, x + 1}, {x - 1, x}};
    std::cout << cp::determinant_integer(a) << "\n";
}
