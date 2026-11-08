#include <iostream>
#include <vector>
#include "../../linear_recurrence/infer_nth.hpp"

int main() {
    std::cout << cp::linear_recurrence_from_prefix(std::vector<int>{0, 1, 1, 2, 3, 5, 8, 13}, 20) << "\n";
}
