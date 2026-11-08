#include <iostream>
#include <vector>
#include "../../computational_geometry/farthest_pair.hpp"

int main() {
    auto pair = cp::geometry::exact::farthest_pair({{0, 0}, {3, 4}, {1, 1}});
    std::cout << static_cast<long long>(pair->squared_distance) << "\n";
}
