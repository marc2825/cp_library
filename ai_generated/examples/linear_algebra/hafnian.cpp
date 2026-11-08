#include "linear_algebra/hafnian.hpp"
#include "basic_utility/big_integer.hpp"
#include <iostream>

template <class T> void print(const std::vector<T> &a) {
    for (std::size_t i = 0; i < a.size(); ++i)
        std::cout << (i ? " " : "") << a[i];
    std::cout << '\n';
}

int main() {
    cp::Matrix<cp::BigInt> a{{0, 2, 3, 5}, {2, 0, 7, 11}, {3, 7, 0, 13}, {5, 11, 13, 0}};
    std::cout << cp::hafnian(a) << "\n";
}
