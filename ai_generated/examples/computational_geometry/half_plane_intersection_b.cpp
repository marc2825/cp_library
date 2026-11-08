#include <iostream>
#include <vector>
#include "../../computational_geometry/half_plane_intersection.hpp"

int main() {
    namespace g = cp::geometry::exact;
    auto polygon = g::half_plane_intersection({g::Line({0, 0}, {1, 0})}, {-1, -1}, {1, 1});
    std::cout << double(g::polygon_area(polygon).approximate()) << "\n";
}
