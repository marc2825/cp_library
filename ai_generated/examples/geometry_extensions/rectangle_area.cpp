#include "computational_geometry/rectangle_union.hpp"
#include <iomanip>
#include <iostream>
using namespace cp::geometry;
int main() {
    std::cout << std::setprecision(6);
    std::cout << exact::rectangle_union_area({{0, 0, 3, 2}, {2, 1, 4, 3}}) << "\n";
}
