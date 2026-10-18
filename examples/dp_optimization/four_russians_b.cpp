#include <iostream>
#include "dp_optimization/four_russians.hpp"
int main() {
    cp::BooleanMatrix a{{1,0,1},{0,1,0}},b{{0,1},{1,0},{1,1}};
    auto c=cp::boolean_matrix_product_four_russians(a,b,2);
    for(const auto& row:c) { for(bool x:row) { std::cout << x; } std::cout << '\n'; }
}
