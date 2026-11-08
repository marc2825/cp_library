#include "computational_geometry/circle_area.hpp"
#include <iomanip>
#include <iostream>
using namespace cp::geometry;
int main() {
    std::cout << std::setprecision(6);
    std::cout << floating::circle_polygon_intersection_area({{0, 0}, 1},
                                                            {{-2, -2}, {2, -2}, {2, 2}, {-2, 2}}) /
                     std::acos(-1.L)
              << "\n";
}
