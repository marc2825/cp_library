#include "computational_geometry/triangle_centers.hpp"
#include <iomanip>
#include <iostream>
using namespace cp::geometry;
int main() {
    std::cout << std::setprecision(6);
    const auto c = *floating::triangle_centers({0, 0}, {4, 0}, {0, 3});
    std::cout << c.circumcircle.radius << " " << c.incircle.radius << " " << c.orthocenter.x << " "
              << c.orthocenter.y << "\n";
}
