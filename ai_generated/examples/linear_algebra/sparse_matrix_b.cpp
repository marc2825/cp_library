#include <iostream>
#include <vector>
#include "../../linear_algebra/sparse_matrix.hpp"

int main() {
    cp::SparseMatrix<int> a(2, 3, {{0, 1, 2}, {0, 1, 3}, {1, 2, -1}});
    auto y = a.multiply({7, 8, 9});
    std::cout << y[0] << " " << y[1] << "\n";
}
