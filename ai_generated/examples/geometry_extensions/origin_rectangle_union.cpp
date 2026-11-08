#include "computational_geometry/origin_rectangle_union.hpp"
#include <iomanip>
#include <iostream>
using namespace cp::geometry;
int main() {
    std::cout << std::setprecision(6);
    exact::OriginRectangleUnion area;
    for (auto p : std::vector<std::pair<int, int>>{{2, 3}, {4, 1}, {3, 2}})
        std::cout << area.insert(p.first, p.second) << " ";
    std::cout << "\n";
}
