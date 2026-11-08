#include "computational_geometry/circle_area.hpp"
#include <iomanip>
#include <iostream>
using namespace cp::geometry;
int main() {
    std::cout << std::setprecision(6);
    std::cout << floating::circle_union_area({{{0, 0}, 1}, {{4, 0}, 1}}) / std::acos(-1.L) << "\n";
}
