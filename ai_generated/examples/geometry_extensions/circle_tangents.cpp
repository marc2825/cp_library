#include "computational_geometry/circle_tangents.hpp"
#include <iomanip>
#include <iostream>
using namespace cp::geometry;
int main() {
    std::cout << std::setprecision(6);
    const auto t = floating::common_tangents({{0, 0}, 1}, {{4, 0}, 1});
    std::cout << t.infinite << " " << t.lines.size() << "\n";
}
