#include "computational_geometry/rational_lattice.hpp"
#include <iomanip>
#include <iostream>
using namespace cp::geometry;
int main() {
    std::cout << std::setprecision(6);
    std::vector<rational::Point> p{{0, 0}, {3, 0}, {3, 2}, {0, 2}};
    std::cout << rational::count_lattice_points(p) << " " << rational::integer_hull(p).size() << "\n";
}
