#include <iostream>
#include <vector>
#include "../../computational_geometry/minkowski_sum.hpp"

int main() {
    namespace g = cp::geometry::exact;
    auto hull = g::minkowski_sum({{0, 0}, {1, 0}, {0, 1}}, {{2, 3}});
    std::cout << hull[0].x << " " << hull[0].y << " " << hull.size() << "\n";
}
