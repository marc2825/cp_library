#include "computational_geometry/convex_farthest_neighbors.hpp"
#include <iomanip>
#include <iostream>
using namespace cp::geometry;
int main() {
    std::cout << std::setprecision(6);
    for (const auto &p : exact::convex_farthest_neighbors({{0, 0}, {4, 0}, {3, 2}, {0, 3}}))
        std::cout << p.squared_distance << " ";
    std::cout << "\n";
}
