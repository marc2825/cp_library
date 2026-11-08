#include "computational_geometry/geometry3d.hpp"
#include <iomanip>
#include <iostream>
using namespace cp::geometry;
int main() {
    std::cout << std::setprecision(6);
    std::vector<spatial::exact::Point> points;
    for (int x : {0, 1})
        for (int y : {0, 1})
            for (int z : {0, 1})
                points.push_back({x, y, z});
    auto h = spatial::exact::convex_hull(points);
    std::cout << h.dimension << " " << h.faces.size() << " " << rational::approximate(h.volume()) << "\n";
}
