#include "computational_geometry/convex_polygon_intersection.hpp"
#include <iomanip>
#include <iostream>
using namespace cp::geometry;
int main() {
    std::cout << std::setprecision(6);
    auto p = rational::convex_polygon_intersection({{0, 0}, {3, 0}, {3, 3}, {0, 3}},
                                                   {{2, -1}, {4, -1}, {4, 2}, {2, 2}});
    std::cout << p.size() << " " << rational::approximate(cp::abs(rational::signed_area2(p)) / 2) << "\n";
}
