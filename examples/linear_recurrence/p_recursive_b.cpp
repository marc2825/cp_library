#include <iostream>
#include <vector>
#include "../../linear_recurrence/p_recursive.hpp"

int main() {
    auto a = cp::p_recursive_prefix(std::vector<int>{1}, std::vector<std::vector<int>>{{-1, -1}, {1}}, 6);
    std::cout << a[5] << "\n";
}
