#include <iostream>
#include <vector>
#include "../../fps/sparse_fps.hpp"

int main() {
    auto c = cp::sparse_fps_inv(cp::SparseFPS{{1, -1}, {0, 1}}, 5);
    std::cout << c[4] << "\n";
}
