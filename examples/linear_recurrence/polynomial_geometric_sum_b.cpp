#include <iostream>
#include <vector>
#include "../../linear_recurrence/polynomial_geometric_sum.hpp"

int main() {
    std::cout << cp::polynomial_geometric_sum(std::vector<int>{0, 1}, 2, 4) << "\n";
}
