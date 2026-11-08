#include <iostream>
#include <vector>
#include "../../fps/compositional_inverse.hpp"

int main() {
    auto g = cp::fps_compositional_inverse(std::vector<int>{0, 1, 1}, 4);
    std::cout << g[1] << " " << g[2] << " " << g[3] << "\n";
}
