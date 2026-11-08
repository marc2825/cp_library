#include <iostream>
#include <vector>
#include "../../fps/composition.hpp"

int main() {
    auto c = cp::fps_compose(std::vector<int>{1, 2, 3}, std::vector<int>{1, 1}, 3);
    std::cout << c[0] << " " << c[1] << " " << c[2] << "\n";
}
