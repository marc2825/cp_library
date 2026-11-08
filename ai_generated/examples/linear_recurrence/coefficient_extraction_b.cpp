#include <iostream>
#include <vector>
#include "../../linear_recurrence/coefficient_extraction.hpp"

int main() {
    std::cout << cp::product_coefficient(std::vector<std::vector<int>>{{1, 1}, {1, 1}, {1, 1}}, 2) << "\n";
}
