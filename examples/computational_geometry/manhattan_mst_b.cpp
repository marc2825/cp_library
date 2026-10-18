#include <iostream>
#include <vector>
#include "../../computational_geometry/manhattan_mst.hpp"

int main() {
    auto result = cp::manhattan_mst({{0, 0}, {1, 0}, {1, 2}});
    std::cout << result.cost << "\n";
}
