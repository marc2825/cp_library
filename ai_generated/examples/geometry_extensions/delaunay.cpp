#include "computational_geometry/delaunay.hpp"
#include <iomanip>
#include <iostream>
using namespace cp::geometry;
int main() {
    std::cout << std::setprecision(6);
    auto d = rational::delaunay_triangulation({{0, 0}, {4, 0}, {4, 4}, {0, 4}, {1, 1}});
    std::cout << d.vertices.size() << " " << d.triangles.size() << " " << d.edges.size() << "\n";
}
