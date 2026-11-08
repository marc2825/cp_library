#include <iostream>
#include <vector>
#include "../../linear_algebra/low_rank_update.hpp"

int main() {
    auto inverse = cp::Matrix<int>::identity(2);
    cp::Matrix<int> u{{1}, {0}}, v{{2}, {0}};
    std::cout << cp::low_rank_determinant_ratio_mod(inverse, u, v) << "\n";
}
