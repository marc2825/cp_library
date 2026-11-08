#include "computational_geometry/segment_arrangement.hpp"
#include <iomanip>
#include <iostream>
using namespace cp::geometry;
int main() {
    std::cout << std::setprecision(6);
    auto a = rational::segment_arrangement({{{0, 0}, {4, 4}}, {{0, 4}, {4, 0}}, {{0, 2}, {4, 2}}});
    std::cout << a.vertices.size() << " " << a.edges.size() << "\n";
}
