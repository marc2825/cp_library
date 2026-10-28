#include "computational_geometry/triangle_point_count.hpp"
#include <iomanip>
#include <iostream>
using namespace cp::geometry;
int main() {
    std::cout << std::setprecision(6);
    exact::TrianglePointCounter p({{0, 0}, {1, 1}, {2, 0}, {3, 3}, {1, 0}});
    std::cout << p.count({0, 0}, {3, 0}, {0, 3}) << " " << p.count({0, 0}, {3, 0}, {0, 3}, false) << "\n";
}
