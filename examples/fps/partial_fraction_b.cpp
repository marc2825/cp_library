#include <iostream>
#include <vector>
#include "../../fps/partial_fraction.hpp"

int main() {
    auto parts = cp::partial_fraction_decomposition(std::vector<int>{1}, {1}, {2});
    std::cout << parts.coefficients[0][0] << " " << parts.coefficients[0][1] << "\n";
}
