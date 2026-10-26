#include <iostream>
#include "combinatorics/combinatorial_transforms.hpp"
int main() {
    const std::vector<int> a{0, 1, 1, 1, 1, 1};
    const auto lah = cp::lah_transform(a);
    const auto euler = cp::euler_transform(a);
    const auto original = cp::euler_transform(euler, true);
    for (const auto& row : {lah, euler, original}) {
        for (std::size_t i = 0; i < row.size(); ++i) std::cout << row[i] << (i + 1 == row.size() ? '\n' : ' ');
    }
}
