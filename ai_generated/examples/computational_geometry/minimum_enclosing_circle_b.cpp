#include <iostream>
#include <vector>
#include "../../computational_geometry/minimum_enclosing_circle.hpp"

int main() {
    namespace g = cp::geometry::floating;
    auto circle = g::minimum_enclosing_circle({{0, 0}, {2, 0}, {0, 2}});
    std::cout << double(circle->center.x) << " " << double(circle->center.y) << "\n";
}
