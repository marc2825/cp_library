#include "linear_algebra/characteristic_polynomial_fast.hpp"
#include <iostream>

template <class T> void print(const std::vector<T> &a) {
    for (std::size_t i = 0; i < a.size(); ++i)
        std::cout << (i ? " " : "") << a[i];
    std::cout << '\n';
}

int main() {
    auto r = cp::characteristic_polynomial_fast(cp::Matrix<int>{{1, 2}, {3, 4}}, {2, 42, 1});
    print(r.coefficients);
}
