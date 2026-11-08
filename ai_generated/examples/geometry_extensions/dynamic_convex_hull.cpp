#include "computational_geometry/dynamic_convex_hull.hpp"
#include <iomanip>
#include <iostream>
using namespace cp::geometry;
int main() {
    std::cout << std::setprecision(6);
    exact::DynamicConvexHull h(2);
    for (auto p : std::vector<exact::Point>{{0, 0}, {4, 0}, {0, 3}, {1, 1}})
        h.insert(p);
    h.erase({1, 1});
    auto p = *h.support({1, 1});
    std::cout << h.hull().size() << " " << p.x << " " << p.y << "\n";
}
