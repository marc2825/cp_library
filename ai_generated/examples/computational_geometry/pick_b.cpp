#include <iostream>
#include <vector>
#include "../../computational_geometry/pick.hpp"

int main() {
    auto count = cp::geometry::exact::lattice_polygon_count({{0, 0}, {3, 0}, {3, 2}, {0, 2}});
    std::cout << static_cast<long long>(count.interior) << " " << static_cast<long long>(count.boundary) << "\n";
}
