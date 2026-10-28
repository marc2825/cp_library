#include "computational_geometry/polygon_triangulation.hpp"
#include <iomanip>
#include <iostream>
using namespace cp::geometry;
int main() {
    std::cout << std::setprecision(6);
    std::vector<rational::Point> p{{0, 0}, {4, 0}, {4, 4}, {2, 2}, {0, 4}};
    auto triangles = rational::triangulate_polygon(p);
    cp::BigRational area;
    for (auto t : triangles)
        area += cp::abs(rational::cross(p[t[0]], p[t[1]], p[t[2]])) / 2;
    std::cout << triangles.size() << " " << rational::approximate(area) << "\n";
}
