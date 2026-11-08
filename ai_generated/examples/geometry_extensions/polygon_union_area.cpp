#include "computational_geometry/polygon_union_area.hpp"
#include <iomanip>
#include <iostream>
using namespace cp::geometry;
int main() {
    std::cout << std::setprecision(6);
    std::vector<std::vector<rational::Point>> p{{{0, 0}, {3, 0}, {3, 2}, {0, 2}},
                                                {{2, 1}, {4, 1}, {4, 3}, {2, 3}}};
    std::cout << rational::approximate(rational::polygon_union_area(p)) << "\n";
}
