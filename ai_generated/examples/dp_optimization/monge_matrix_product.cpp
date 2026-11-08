#include <iostream>
#include "dp_optimization/monge_matrix_product.hpp"
int main() {
    std::vector<std::vector<long long>> a{{0,4,2},{3,0,5}};
    auto result = cp::monge_min_plus_product(2,3,3,
        [&](int i,int k) { return a[i][k]; }, [](int k,int j) { return 1LL*(k-j)*(k-j); });
    for (const auto& row : result.value) {
        for (std::size_t j = 0; j < row.size(); ++j) std::cout << (j ? " " : "") << *row[j];
        std::cout << '\n';
    }
}
