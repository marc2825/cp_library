#include <iostream>
#include "linear_algebra/matrix.hpp"
int main() {
    cp::Matrix<int> a{{1, 2, 3}, {4, 5, 6}};
    auto t = a.transposed();
    std::cout << a.rows() << ' ' << a.cols() << '\n';
    std::cout << t(2, 1) << ' ' << cp::Matrix<int>::identity(2)(1, 1) << '\n';
}
