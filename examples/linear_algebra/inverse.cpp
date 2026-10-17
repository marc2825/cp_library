#include <iostream>
#include "linear_algebra/inverse.hpp"
int main() {
    auto b = cp::matrix_inverse_mod(cp::Matrix<int>{{1, 2}, {3, 4}});
    if (b) for (std::size_t i = 0; i < b->rows(); ++i) {
        for (std::size_t j = 0; j < b->cols(); ++j) std::cout << (*b)(i, j) << ' ';
        std::cout << '\n';
    }
}
