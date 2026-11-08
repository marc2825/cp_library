#include "computational_geometry/voronoi.hpp"
#include <iomanip>
#include <iostream>
using namespace cp::geometry;
int main() {
    std::cout << std::setprecision(6);
    auto d = rational::voronoi_diagram({{1, 1}, {3, 1}, {2, 3}});
    auto cells = rational::voronoi_cells(d, {0, 0}, {4, 4});
    cp::BigRational area;
    for (const auto &p : cells)
        area += cp::abs(rational::signed_area2(p)) / 2;
    std::cout << cells.size() << " " << rational::approximate(area) << "\n";
}
