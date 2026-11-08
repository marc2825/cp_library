#include "computational_geometry/polygon_centroid.hpp"
#include <iomanip>
#include <iostream>
using namespace cp::geometry;
int main() {
    std::cout << std::setprecision(6);
    const auto p = *rational::polygon_centroid({{0, 0}, {4, 0}, {4, 2}, {0, 2}});
    std::cout << rational::approximate(p.x) << " " << rational::approximate(p.y) << "\n";
}
