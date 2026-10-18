#include <iostream>
#include <vector>
#include "../../linear_algebra/black_box.hpp"

int main() {
    auto apply = [](const std::vector<int>& x) { return x; };
    auto y = cp::black_box_power_apply(2, apply, std::vector<int>{2, 3}, 100, std::vector<int>{-1, 1});
    std::cout << y[0] << " " << y[1] << "\n";
}
