#include <iostream>
#include <vector>
#include "linear_algebra/matrix_product.hpp"
int main() {
    cp::Matrix<long long> a{{1, 2, 3}, {4, 5, 6}}, b{{7, 8}, {9, 10}, {11, 12}};
    auto c = a * b;
    for (std::size_t i = 0; i < c.rows(); ++i) {
        for (std::size_t j = 0; j < c.cols(); ++j) std::cout << c(i, j) << ' ';
        std::cout << '\n';
    }
    auto y = cp::matrix_vector_multiply(a, std::vector<long long>{1, 0, -1});
    for (auto x : y) std::cout << x << ' ';
    std::cout << '\n';
}
