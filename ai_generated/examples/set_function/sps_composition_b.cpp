#include <iostream>
#include <vector>
#include "../../set_function/sps_composition.hpp"

int main() {
    auto c = cp::sps_compose(std::vector<int>{1, 2, 1}, std::vector<int>{0, 1, 1, 0});
    std::cout << c[0] << " " << c[3] << "\n";
}
