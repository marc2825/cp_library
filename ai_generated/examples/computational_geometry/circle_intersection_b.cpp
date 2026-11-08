#include <iostream>
#include <vector>
#include "../../computational_geometry/circle_intersection.hpp"

int main() {
    namespace g = cp::geometry::floating;
    auto result = g::circle_circle_intersection(g::Circle({0, 0}, 1), g::Circle({2, 0}, 1));
    std::cout << result.points.size() << " " << double(result.points[0].x) << "\n";
}
