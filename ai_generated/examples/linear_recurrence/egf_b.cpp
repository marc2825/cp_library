#include <iostream>
#include <vector>
#include "../../linear_recurrence/egf.hpp"

int main() {
    auto c = cp::egf_multiply(std::vector<int>(5, 1), std::vector<int>(5, 1), 5);
    std::cout << c[4] << "\n";
}
