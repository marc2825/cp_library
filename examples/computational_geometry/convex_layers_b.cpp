#include <iostream>
#include <vector>
#include "../../computational_geometry/convex_layers.hpp"

int main() {
    auto layer = cp::geometry::exact::convex_layers({{0, 0}, {4, 0}, {4, 4}, {0, 4}, {2, 2}});
    std::cout << layer[0] << " " << layer[4] << "\n";
}
