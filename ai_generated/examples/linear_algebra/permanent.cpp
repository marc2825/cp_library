#include "linear_algebra/permanent.hpp"
#include "basic_utility/big_integer.hpp"
#include <iostream>

template <class T> void print(const std::vector<T> &a) {
    for (std::size_t i = 0; i < a.size(); ++i)
        std::cout << (i ? " " : "") << a[i];
    std::cout << '\n';
}

int main() {
    cp::Matrix<cp::BigInt> a{{1, 2, 0}, {0, 3, 4}, {5, 0, 6}};
    std::cout << cp::permanent(a) << "\n";
}
